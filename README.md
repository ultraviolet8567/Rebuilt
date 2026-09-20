# Bugfix Branch: `bugfix/shooter-seperate-fixes`

This branch is a code review of the `shooter-seperate` branch, carried out
during the 2026 season and kept rebased on top of the team's latest work
(currently `ad8a011`, the post-BattleCry "Fixed Demo Mode" commit). Every
change here is something we believe is a bug in `shooter-seperate`; nothing
on this branch adds features or re-tunes values that were calibrated on the
robot.

**How to review:** look at the commits one at a time (each has a detailed
message), or see everything at once:

```
git diff shooter-seperate..bugfix/shooter-seperate-fixes
```

**How to take the fixes:** merge the branch, or cherry-pick individual
commits if you only want some of them. If `shooter-seperate` has moved on
again, the branch needs a rebase first.

---

## Critical fixes (robot crashes or majorly wrong behavior)

### 1. Robot code can crash when no alliance is assigned — `Odometry.java`

`resetHeading()` and `getHub()` both called
`DriverStation.getAlliance().get()` without checking that an alliance
exists. `getAlliance()` returns an `Optional` — it is **empty** until the
robot is connected to the Driver Station / FMS and an alliance is assigned.
Calling `.get()` on an empty Optional throws `NoSuchElementException`.

This is worse than it sounds because `resetHeading()` is called from the
`Odometry` **constructor**, which runs the moment the robot code boots —
usually *before* the Driver Station has attached. Depending on timing, the
robot program can crash during startup, in the pit, or on the practice
field. The driver's Back button (gyro reset) hits the same code.

**Fix:** check `isPresent()` first. `resetHeading()` falls back to a plain
`gyro.reset()` when no alliance is known; `getHub()` defaults to the blue
hub.

### 2. Field-oriented drive was never actually flipped for the Red alliance — `ManualTeleOp.java`, `DriftTeleOp.java`

The heading used for field-oriented driving was transformed twice:

```java
Rotation2d currentHeading = odometry.getHeading();
if (DriverStation.getAlliance().get() == Alliance.Red)
    currentHeading = currentHeading.unaryMinus();          // flip #1
currentHeading = AllianceFlipUtil.apply(currentHeading);   // flip #2 (also flips on Red!)
```

`AllianceFlipUtil.apply()` *already* flips the rotation when on the Red
alliance. Negating it manually first means the two flips cancel out — on
Red the heading was effectively never flipped at all, and on Blue the code
did nothing. So Red-alliance field-oriented driving used the wrong frame
all season. (This also contained the same unguarded `.get()` crash as #1.)

**Fix:** a single `AllianceFlipUtil.apply(...)` call. `DriftTeleOp` keeps
the team's choice of `getGyrometerHeading()` as the source.

### 3. Demo mode change broke ALL shooting — `Flywheel.java`

The "Fixed Demo Mode" commit turned `getTargetVelocity()` from a getter
into something that **overwrites** the target every time it is called:

```java
public double getTargetVelocity() {
    if (isDemo)  targetVelocity = kFlywheelMaxVelocity * demoScale;
    else         targetVelocity = kFlywheelMaxVelocity;     // overwrites the field!
    return targetVelocity;
}
```

`periodic()` calls this every 20 ms. So one cycle after any command sets a
velocity — `CalculatedShoot`'s distance-based value, or `Shuffle`'s
dashboard tunable (5200) — it gets clobbered back to
`kFlywheelMaxVelocity` (2200). Every shot would spin at 2200 no matter
what was commanded. The distance lookup table and the dashboard tunable
were both effectively dead code.

**Fix:** the getter no longer assigns. Demo scaling is applied to the
*commanded* velocity on read:

```java
return isDemo ? targetVelocity * demoScale : targetVelocity;
```

Side benefit: demo mode now also scales *calculated* shots, not just the
constant.

---

## Command scheduler fixes

### 4. Nine commands never declared their subsystem requirements

In WPILib's command framework, `addRequirements(subsystem)` is how the
scheduler knows two commands conflict. If a command uses a subsystem
without requiring it, the scheduler will happily run it **at the same
time** as another command on the same subsystem, and the two fight over
the motors — the winner is whichever happened to set the output last each
cycle. The symptom is intermittent, "possessed" mechanism behavior that is
very hard to debug at competition.

Commands fixed (requirement added in parentheses):

| Command | Now requires |
|---|---|
| `Shoot` | flywheel |
| `DirectShoot` | flywheel |
| `DirectHood` | hood |
| `SetHood` | hood |
| `SetPivot` | pivot |
| `ManualKicker` | kicker |
| `CalculatedShoot` | flywheel, indexer |
| `Shuffle` | flywheel, hood, indexer |
| `Feed` | pivot |

Concrete example: the operator can hold POV-left (`Feed`, oscillates the
pivot) and press Y (`SetPivot`) at the same time. Without requirements both
run simultaneously and the pivot target flips back and forth between two
commands' wishes. With requirements, scheduling the second command
automatically cancels the first.

`SetHood` also gained an `end()` that stops the hood motor — previously
releasing the POV button left the hood holding its last commanded output.

### 5. `CalculatedShoot` only computed the shot velocity once

The distance-based flywheel velocity was computed in `initialize()` (the
moment the trigger is pressed) and never updated. If the robot moved while
the operator held the trigger, the flywheel speed went stale. An
`execute()` override now recalculates the target from the live
`distToHub()` every cycle.

---

## Smaller functional fixes

### 6. Low-battery indicator lit the whole LED strip — `Lights.java`

The `Section` enum's `start()`/`end()` had been flattened so every section
returned the full strip range — `Section.BOTTOM` and `Section.UPPER` were
silently identical to `FULL`. The low-battery warning
(`strobe(Section.BOTTOM, ...)`) therefore strobed all 42 LEDs instead of
just the bottom 8, hiding whatever the main pattern was showing. The
original switch statements (with `bottomLength = 8`) are restored.

### 7. Autonomous LED animation was frozen — `Lights.java`

```java
breath(Section.FULL, Color.kRed, Color.kBlue, 4, 1);
```

The 5th argument of that `breath()` overload is a **timestamp**, not a
speed. Passing the literal `1` evaluated the animation at one fixed instant
forever — auto showed a single static color. The call now uses the 4-arg
overload, which reads the live FPGA clock.

### 8. Selecting a non-existent auto crashed with NullPointerException — `AutoChooser.java`

`getSelectedAuto()` and `getAutoStartingPose()` guarded against the name
`"Do Nothing"` — but the chooser never produces that string. The actual
name is built as `driveOut + " " + side`, so the default ("None" + "None")
produces `" "` and unmatched combinations produce names that aren't in the
`allAutos` map. `allAutos.get(...)` then returns `null`, and
`autonomousInit` (or `.getStartingPose()`) throws an NPE. Both methods now
null-check the lookup. The team's `Autos/StartPos` logging is kept.

### 9. Inversion ternaries were backwards — `Funnel.java`, `Indexer.java`, `Constants.java`

Both used `voltage *= inverted ? 1 : -1` — i.e. `inverted = true` meant *no*
inversion, the opposite of every other motor in the codebase and a trap for
the next person who edits it. The ternaries are corrected to the
conventional `inverted ? -1 : 1`, **and** `kFunnelInverted` /
`kIndexerInverted` are flipped from `true` to `false` at the same time, so
the net motor direction is exactly what the team has been running at
competitions. (Fixing only the ternary would have reversed both motors.)

### 10. Flywheel cleanups — `Flywheel.java`

- The follower motor's feedforward now uses the same velocity parameter as
  the lead motor's (it was calling `getTargetVelocity()`, which under the
  old mutating getter could differ from the parameter).
- Removed an unused `edu.wpi.first.units.measure.Velocity` import.

---

## Documentation-only changes (no behavior change — decisions for the team)

### 11. The shooter lookup table decreases beyond ~5 m — TODO in `Flywheel.java`

```java
return 1310.13 + 1270.33 * dist - 128.09 * dist * dist;
```

This parabola peaks at `dist = 1270.33 / (2 × 128.09) ≈ 4.96 m` and then
**goes down**:

| distance | commanded velocity |
|---|---|
| 2 m | 3338 |
| 4 m | 4342 |
| **5 m** | **4460 (peak)** |
| 7 m | 3926 |
| 8 m | 3275 |

So a shot from 7 m is commanded *slower* than one from 5 m — far shots will
undershoot. The curve was probably fit on data points all within ~2–5 m,
where it is fine; the quadratic just extrapolates badly. Options (pick
one): refit with far-range data points, switch to the logarithmic
alternative already sitting in a comment below it (monotonic — always
increases), or clamp the input distance at the peak. A TODO comment with
these numbers is in the code.

### 12. Absolute encoder offset bookkeeping — TODO in `Constants.java`

The drive encoder offsets are sums of accumulated corrections whose inline
comments no longer match the computed values, and some land outside
[-π, π] before wrapping. They *work* (they've been re-trimmed at every
event, most recently BattleCry), but nobody can tell anymore what the
"true" measured zero of each module is. The TODO describes a clean
recalibration procedure: point all wheels straight forward, read each
absolute encoder's raw voltage, convert to radians (voltage / 5 V × 2π),
and use the negation of that as the single offset.

---

## Fixes that were on this branch earlier and have been REMOVED (and why)

These are worth knowing about so nobody re-introduces them:

- **Wheel-twitch-at-rest workaround in `Swerve.setModuleStates`** — dropped.
  The team's BattleCry rework (dedicated `setModuleRotation()` path for
  `lockWheels()`, turning PID back inside the speed check) fixed the
  underlying problem properly, making the workaround dead code.

- **`Shooter.periodic()` / `Intake.periodic()` delegation to
  Flywheel/Hood/Pivot** — dropped, because the diagnosis behind it was
  wrong. `SubsystemBase`'s constructor automatically registers every
  subsystem with the `CommandScheduler` (verified in the WPILib 2026.2.1
  bytecode), so nested subsystems like `Flywheel` already get their
  `periodic()` called by the scheduler. The delegation made them run
  **twice per loop**, which doubles PID integral accumulation and corrupts
  the derivative term. Lesson for future architecture: anything extending
  `SubsystemBase` gets `periodic()` for free, no wiring needed.

---

## Open items we did NOT change (worth verifying on the robot)

1. **Follower flywheel PID negation** — `setFlywheelRadsPerSec` feeds the
   PID `-getVelocity(followerMotor)` while the lead uses the positive
   velocity. This is correct *only if* the follower's encoder genuinely
   reads negative while shooting (mirrored mounting). One log check
   settles it: `Shooter/Flywheel/Follower/Velocity` should be **negative**
   during a shot. If it's positive, the follower PID is fighting its own
   feedforward. Left as-is because the current behavior is field-tested.

2. **Demo speed constants have crossed units** — in `DriveConstants`:
   `kDemoTeleDriveMaxSpeedMetersPerSecond` is derived from the *angular*
   speed constant and `kDemoTeleDriveMaxAngularSpeedRadiansPerSecond` from
   the *linear* one. The numbers happen to come out usable (~2.7 m/s), but
   the derivation is wrong and will bite whoever edits it next.

3. **Slew-rate limiters are disabled in `ManualTeleOp`** (commented out
   since DCMP) with the tele max speed raised to 5 m/s. Instant
   acceleration commands cause wheel slip, and wheel slip corrupts the
   odometry that `CalculatedShoot` depends on. If shot accuracy degrades
   again, this is a prime suspect.

4. **Vision rotation stddev is 0.5** in the pose estimator. With MegaTag2
   the pose rotation effectively *is* the gyro, so fusing vision rotation
   tightly lets it fight the gyro. Common practice is to set the rotation
   stddev huge (e.g. 9999999) so only vision x/y are fused.

5. **AdvantageKit logs write to `/home/lvuser/logs/`** (roboRIO internal
   flash) since DCMP. Internal flash is small; logs can fill it over a
   long event and break deploys. Clean it out periodically or move logging
   back to a USB stick.

---

## Commit map

| Commit | Contents |
|---|---|
| `Fix critical swerve, command scheduler, and subsystem bugs` | Items 1, 2, 4 (most commands), 5, 8, 9 (ternaries), 12 |
| `Fix issues introduced in DCMP/WPI testing commits` | Items 4 (Shuffle/CalculatedShoot indexer+hood), 6, 11 |
| `Fix critical demo-mode regression and BattleCry command bugs` | Items 3, 4 (Feed), 7, 10 |
| `Correct two earlier fixes on this branch` | Removed double-periodic delegation; flipped inversion constants (item 9) |
