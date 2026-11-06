# Controls

What every controller button does, and why. Generated from
`src/main/java/frc/robot/RobotContainer.java`; update this file when you change a binding.

Both controllers are Xbox-style (`OperatorConstants.controllerTypeDriver/Operator = XBOX`).

## Driver controller (port 0)

| Input | Type | Action | Code |
| --- | --- | --- | --- |
| Left stick Y/X | analog | Field-relative translation. Full speed. | `ManualTeleOp` (default command on `Swerve`) |
| Right stick X | analog | Rotation. | `ManualTeleOp` |
| Right bumper | hold | **Slow mode**: all speeds scaled to 33%, light rumble as feedback. | `ManualTeleOp` |
| X | hold | **Lock wheels** into an X pattern; drive input is ignored. | `ManualTeleOp` |
| Back | press | Reset odometry heading to zero (field-relative forward). | `odometry.resetHeading()` |
| Right trigger | hold | **Hub-lock driving**: same sticks, but rotation is PID-controlled to face the hub. Use while driving and shooting. | `DriftTeleOp` |

Stick directions are negated on the way in (WPILib reports forward stick as negative), so pushing the
left stick away from you drives the robot away from you.

Field-relative heading is alliance-corrected through `AllianceFlipUtil`, so "away from you" means the
same thing on both sides of the field.

`DriftTeleOp` cancels `ManualTeleOp` while held because both require `Swerve`; releasing the trigger
hands control back to the default command.

## Operator controller (port 1)

### Intake and storage

| Input | Type | Action | Code |
| --- | --- | --- | --- |
| Left bumper | hold | Run the funnel forward to collect. | `SpinIntake(funnel, false)` |
| B | hold | Run the funnel **in reverse** to eject or clear a jam. | `SpinIntake(funnel, true)` |
| Left trigger | hold | Run the indexer forward. | `SpinIndexer(indexer, false)` |

### Pivot setpoints

| Input | Type | Target | Code |
| --- | --- | --- | --- |
| Y | hold | `IntakeConstants.kPivotLower` (0.1) | `SetPivot` |
| X | hold | `IntakeConstants.kPivotMiddle` (0.9) | `SetPivot` |
| A | hold | `IntakeConstants.kPivotUpper` (2.0) | `SetPivot` |
| POV left | hold | **Feed**: oscillates between lower and upper every ~25 loops (0.5 s at 50 Hz) | `Feed` |

`SetPivot` only sets the target in `initialize()`, so these are "hold to keep the setpoint alive,
release to let another command take the pivot".

### Shooter

| Input | Type | Action | Code |
| --- | --- | --- | --- |
| Right trigger | hold | **Calculated shot**: flywheel speed from the measured distance to the hub, plus indexer. | `CalculatedShoot` |
| Right bumper | hold | **Fixed shot**: flywheel to `ShooterConstants.kFlywheelVelocity` (tunable, default 5200), hood to `kHoodUpper`, plus indexer. | `Shuffle` |
| POV up | hold | Nudge the hood target up by 0.01 per loop. | `SetHood(hood, false)` |
| POV down | hold | Nudge the hood target down by 0.01 per loop. | `SetHood(hood, true)` |

All shoot commands stop the flywheel, hood, and indexer on release, including when interrupted.

### Kicker

The kicker has a **default command**, `RunKicker`, so it runs whenever no other command requires it.
It spins only while the flywheel is at its target velocity:

```java
if (flywheel.atVelocity(flywheel.getTargetVelocity())) { kicker.start(); } else { kicker.stop(); }
```

There is no button for it. To stop feeding balls you stop the flywheel, not the kicker. Commented-out
`ManualKicker` bindings exist in `RobotContainer` and are not active.

## PathPlanner named commands

Registered in `RobotContainer`; these exact strings are referenced by `.auto` files. Renaming one
silently breaks every auto that uses it.

| Name | Command | Notes |
| --- | --- | --- |
| `Intake` | `SpinIntake(funnel, false)` | |
| `Index` | `SpinIndexer(indexer, false)` | |
| `ShootMiddle` | `Shoot(flywheel, 3000)` | Fixed 3000 velocity |
| `PivotUp` | `SetPivot(pivot, kPivotLower)` | **Naming is inverted**: "Up" maps to the *lower* constant |
| `PivotMiddle` | `SetPivot(pivot, kPivotMiddle)` | |
| `PivotDown` | `SetPivot(pivot, kPivotUpper)` | **Naming is inverted**: "Down" maps to the *upper* constant |
| `Feed` | `Feed(pivot)` | Oscillating feed |
| `CalculatedShoot` | `CalculatedShoot(flywheel, odometry, indexer)` | Distance-based |
| `Shoot` | `Shuffle(flywheel, hood, indexer)` | Fixed shot plus hood |
| `AutoAlign` | `DriftTeleOp(..., targetAngle = -angleToHub)` | Hub-facing while driving |

## Known rough edges

- `PivotUp`/`PivotDown` map to `kPivotLower`/`kPivotUpper` the wrong way round. Autos that look
  right in PathPlanner can therefore move the pivot the opposite way. Verify on the robot before
  trusting an auto's pivot step.
- `Constants.OIConstants` duplicates `OperatorConstants` controller ports (both 0 and 1). They agree
  today; if one changes, the other does not.
- `ManualTeleOp` and `DriftTeleOp` call `AllianceFlipUtil.apply()` on a heading they have already
  negated for red, so the red-side heading is flipped twice. Worth checking against a real match
  before changing, since the robot drives correctly today.
