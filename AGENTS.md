# Rebuilt — FRC Team 8567 robot code

Java 17 command-based FRC robot project (WPILib 2026, GradleRIO 2026.2.1). Team number is
`8567` (from `.wpilib/wpilib_preferences.json`). Main class: `frc.robot.Main`.

## Build and verify

Run from the repo root. In PowerShell use `.\gradlew.bat`; in bash use `./gradlew`.

This shell has no JDK on `PATH`. Set the WPILib JDK first (the WPILib VS Code extension does it
automatically):

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'
```

| Task | Command |
| --- | --- |
| Compile only (fast sanity check) | `.\gradlew.bat compileJava` |
| Full build (compile + `spotlessCheck` + jar) | `.\gradlew.bat build` |
| Auto-format Java/Gradle/XML/Markdown | `.\gradlew.bat spotlessApply` |
| Unit tests | `.\gradlew.bat test` |
| Simulation (GUI + driver station) | `.\gradlew.bat simulateJava` |
| Deploy to the roboRIO | `.\gradlew.bat deploy` |

`build` runs `spotlessCheck`, so formatting failures break the build. Run `spotlessApply` after
edits rather than hand-formatting.

There is no `src/test` directory yet. JUnit 5 is already on the test classpath (`test` uses
`useJUnitPlatform()`), so new tests belong in `src/test/java` under the same package.

Never run `deploy` unless the user explicitly asks and the robot is safe to enable.

## Layout

```
src/main/java/frc/robot/
  Main.java, Robot.java, RobotContainer.java   entry point, lifecycle, all bindings
  Constants.java                               every tunable and CAN ID
  LimelightHelpers.java                        vendored Limelight utility
  commands/                                    one command per file
  subsystems/                                  Swerve, Odometry, SwerveModule, Lights, AutoChooser
    Intake/    Funnel, Intake, Pivot
    Shooter/   Flywheel, Hood, Kicker, Shooter
    Storage/   Indexer, Storage
  util/                                        AllianceFlipUtil, LoggedTunableNumber, VirtualSubsystem
src/main/deploy/                               PathPlanner autos/paths, robot config, other assets
vendordeps/                                    AdvantageKit, PathPlanner, Phoenix 6, REVLib, WPILibNewCommands
```

Libraries in use: AdvantageKit (logging), PathPlanner (autos), Phoenix 6 / CTRE (TalonFX),
REVLib (SparkFlex/SparkMax), Limelight, WPILib New Commands.

## Conventions

- **Formatting:** Google Java Format **AOSP** style via Spotless — 4-space indent, 100-column
  limit. Do not reformat unrelated code.
- **Fields:** `private final` for subsystems, motors, controllers, configs. Static subsystem
  access is via singletons (`Lights.getInstance()`).
- **Constants:** all magic numbers live in `Constants.java` as `k`-prefixed values inside nested
  `public static final class` blocks (`ShooterConstants`, `IntakeConstants`, `StorageConstants`,
  `OdometryConstants`, `FieldConstants`, `CAN`, …). Add new tunables there, not inline.
- **Tunables:** use `LoggedTunableNumber` with `.get()` in `periodic()` so dashboard changes take
  effect live (see `Flywheel.periodic()` re-reading PID/FF gains).
- **Logging:** AdvantageKit auto-logging is enabled through the `akit-autolog` annotation
  processor. Explicit values use `Logger.recordOutput("Subsystem/Part/Value", value)` with a
  hierarchical key (e.g. `Shooter/Flywheel/Lead/Velocity`). Keep the naming scheme consistent.
- **Commands:** one command per file in `commands/`, taking the subsystem piece it drives as a
  constructor argument (e.g. `new SpinIntake(intake.getFunnel(), false)`). Bind them in
  `RobotContainer.configureBindings()`; never put robot logic in `Robot.java` periodic methods.
- **Autos:** PathPlanner named commands are registered in `RobotContainer` (`Intake`, `Index`,
  `ShootMiddle`, `PivotUp`, `PivotMiddle`, `PivotDown`, `Feed`, `CalculatedShoot`, `Shoot`,
  `AutoAlign`). Renaming one silently breaks any `.auto` file that references it — update both.
- **Alliances:** use `AllianceFlipUtil` / `DriverStation.getAlliance()` rather than hard-coding
  red/blue geometry.
- **Demo mode:** `Lights.getInstance().isDemo` gates scaled-down setpoints (see `Flywheel`). Keep
  new mechanism code demo-safe instead of adding separate demo classes.

## Hardware and safety

- CAN IDs, motor inversions, gearing, and current limits are in `Constants.CAN` and the
  `*Constants` classes. Do not renumber CAN IDs or change inversions without saying so explicitly —
  a wrong inversion spins a mechanism the wrong way on a real robot.
- Vendor library versions in `vendordeps/` are pinned. Do not bump them unless asked; a mismatch
  with the installed WPILib year breaks the build.
- Prefer simulation (`simulateJava`) or `compileJava` for verification when no robot is connected.
- Never commit `build/`, `bin/`, `.gradle/`, `.wpilib/`, `.DataLogTool/`, logs, or `.wpilib` caches.

## Git

- Remote: `https://github.com/ultraviolet8567/Rebuilt.git`, active branch `shooter-seperate`.
- This is the only working clone. The older clone, which still has an in-progress conflicted
  merge and uncommitted work, is archived at `..\_archive\2026-Rebuilt-old-clone`. Never edit
  the archive; use it only to recover a file.
- `..\README.md` indexes all FRC projects, and the shared config at `..\opencode.jsonc` exposes
  the 2025 Reefscape project as a `reefscape-2025` reference.
- Do not rewrite history, force-push, or merge branches without explicit instruction.
