# CAN map

Every device on the robot bus, generated from `Constants.CAN` and the subsystem constructors. Update
this file when an ID or controller type changes, and never renumber an ID without saying so: a
duplicate ID makes two mechanisms move together.

Legal FRC CAN IDs are 1-62. `ConstantsTest` fails the build if an ID repeats or falls outside that
range.

## Shooter

| ID | Device | Controller | Details |
| --- | --- | --- | --- |
| 1 | Flywheel lead (`kFlywheelLeadPort`) | SparkFlex | 80 A current limit, coast mode, inverted (`kFlywheelInverted = true`), 10 V clamp |
| 2 | Flywheel follower (`kFlywheelFollowerPort`) | SparkFlex | Same config as the lead, **not** a `.follow()` — it runs its own PID/FF loop |
| 3 | Kicker (`kKickerPort`) | SparkMax | 3:1 reduction, 12 V clamp, inverted |
| 4 | Hood (`kHoodPort`) | SparkMax | 25:1 gearbox × 16.8:1 rack, 10 V clamp, inverted |

## Intake

| ID | Device | Controller | Details |
| --- | --- | --- | --- |
| 5 | Pivot (`kPivotPort`) | SparkMax | 45:1 gearbox × 2.5:1 chain, 10 V clamp, soft limits 0.1-2.0 |
| 6 | Funnel (`kFunnelPort`) | **TalonFX** (Phoenix 6) | The only CTRE device on the robot, 12 V clamp, inverted |

## Storage

| ID | Device | Controller | Details |
| --- | --- | --- | --- |
| 7 | Indexer (`kIndexerPort`) | SparkFlex | 8 V clamp, inverted |

## Swerve

| ID | Device | Controller | Details |
| --- | --- | --- | --- |
| 10 | Front left drive | SparkFlex | Drive encoder reversed |
| 11 | Front right drive | SparkFlex | Drive encoder reversed |
| 12 | Back left drive | SparkFlex | Drive encoder reversed |
| 13 | Back right drive | SparkFlex | Drive encoder reversed |
| 20 | Front left turning | SparkMax | |
| 21 | Front right turning | SparkMax | |
| 22 | Back left turning | SparkMax | |
| 23 | Back right turning | SparkMax | |

Drive motors are `SparkFlex`, turning motors are `SparkMax`. The gear ratio comes from
`ModuleConstants.kDriveMotorGearing` (1, 2, or 3), selected in `RobotContainer` when the swerve is
constructed.

## Not on CAN

These use roboRIO I/O pins and may legitimately reuse numbers that look like CAN IDs:

| Device | Port | Type |
| --- | --- | --- |
| Hood absolute encoder (`kHoodEncoderPort`) | DIO 0 | `DutyCycleEncoder` |
| Pivot absolute encoder (`kPivotEncoderPort`) | DIO 1 | `DutyCycleEncoder` |
| Swerve absolute encoders (`kFrontLeft/Right/BackLeft/RightDriveAbsoluteEncoderPort`) | Analog 0-3 | `AnalogInput` |

`Constants.CAN.kHoodEncoderPort = 0` and `kPivotEncoderPort = 1` sit next to CAN IDs 1 and 2 in the
same class. They are **not** on the CAN bus. `ConstantsTest` excludes fields whose names contain
`Encoder` for exactly this reason.

## Controllers

| Device | Port |
| --- | --- |
| Driver Xbox controller | USB 0 |
| Operator Xbox controller | USB 1 |

## Quick check before powering a new mechanism

1. ID is unique and in 1-62.
2. Controller type matches the constant's name (`Flywheel`/`Indexer` are Flex, `Kicker`/`Hood`/
   `Pivot` are Max, `Funnel` is TalonFX).
3. Inversion is unchanged unless you have verified the direction with the robot on blocks.
4. Voltage clamp in `Constants` matches the mechanism's rating.
