package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Driver-facing drivetrain commands.
 *
 * <p>Replaces {@code ManualTeleOp}, {@code DriftTeleOp} and {@code CorrectAuto}. Those three were
 * near-identical 150-line classes -- the heading logic was the only real difference, and the
 * duplicated joystick handling in each had drifted apart, so the two of them treated the deadband,
 * the slow-mode scaling and the alliance flip slightly differently. Everything shared now lives in
 * one place.
 */
public final class DriveCommands {
    private DriveCommands() {}

    private static final double kSlowModeFactor = 0.33;

    /** Rumble level while slow mode is engaged, so the driver can feel that it is on. */
    private static final double kSlowModeRumble = 0.025;

    /**
     * Joystick axes to a field-relative velocity vector.
     *
     * <p>The deadband is applied to the <em>magnitude</em> of the stick vector, not to each axis
     * separately. Per-axis deadbands carve a cross-shaped dead zone out of the stick: pushing
     * diagonally at 0.1 in each axis gives nothing, and a stick that is slightly off-centre on one
     * axis distorts every diagonal. The magnitude is then squared, which gives fine control near
     * centre and full speed at the rim -- the arrangement every corpus robot that does this uses.
     *
     * <p>This also removes the {@code xSpeed *= (xSpeed > 0) ? (1/0.8) : (1/0.9)} scaling the old
     * code applied, which silently made forward 25% more sensitive than backward.
     */
    private static Translation2d linearVelocityFromJoysticks(double x, double y) {
        double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), OIConstants.kDeadband);
        Rotation2d direction = new Rotation2d(Math.atan2(y, x));
        magnitude = magnitude * magnitude;
        return new Pose2d(Translation2d.kZero, direction)
                .transformBy(new Transform2d(magnitude, 0.0, Rotation2d.kZero))
                .getTranslation();
    }

    /** Free driving: the right stick controls yaw rate directly. */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            BooleanSupplier slowMode,
            BooleanSupplier lockWheels) {
        return drive.run(
                        () -> {
                            if (lockWheels.getAsBoolean()) {
                                RobotState.getInstance().setWheelsLocked(true);
                                drive.stopWithX();
                                return;
                            }
                            RobotState.getInstance().setWheelsLocked(false);

                            Translation2d linear =
                                    linearVelocityFromJoysticks(
                                            xSupplier.getAsDouble(), ySupplier.getAsDouble());
                            double omega =
                                    MathUtil.applyDeadband(
                                            omegaSupplier.getAsDouble(), OIConstants.kDeadband);
                            omega = Math.copySign(omega * omega, omega);

                            double scale = slowMode.getAsBoolean() ? kSlowModeFactor : 1.0;
                            drive.runVelocity(
                                    toFieldRelative(
                                            drive,
                                            linear.getX()
                                                    * drive.getMaxLinearSpeedMetersPerSec()
                                                    * scale,
                                            linear.getY()
                                                    * drive.getMaxLinearSpeedMetersPerSec()
                                                    * scale,
                                            omega * drive.getMaxAngularSpeedRadPerSec() * scale));
                        })
                .withName("Joystick drive");
    }

    /**
     * Drive while holding a commanded heading.
     *
     * <p>The old {@code DriftTeleOp} computed the shortest way round by hand, with four branches
     * comparing {@code target} and {@code heading} and adding or subtracting 2*pi. A {@link
     * ProfiledPIDController} with {@code enableContinuousInput} does that correctly by
     * construction, and the trapezoid profile on top of it means the robot slews to the new heading
     * at a bounded rate instead of jerking at whatever the proportional term happens to produce. 34
     * of the 42 corpus robots let the controller handle the wrap.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> targetHeading,
            BooleanSupplier slowMode,
            BooleanSupplier lockWheels) {
        ProfiledPIDController heading =
                new ProfiledPIDController(
                        DriveConstants.kHeadingKp.get(),
                        0.0,
                        DriveConstants.kHeadingKd.get(),
                        new TrapezoidProfile.Constraints(
                                DriveConstants.kHeadingMaxVelRadPerSec,
                                DriveConstants.kHeadingMaxAccelRadPerSecSq));
        heading.enableContinuousInput(-Math.PI, Math.PI);
        heading.setTolerance(DriveConstants.kHeadingToleranceRad);

        return drive.run(
                        () -> {
                            LoggedTunableNumber.ifChanged(
                                    heading.hashCode(),
                                    () ->
                                            heading.setPID(
                                                    DriveConstants.kHeadingKp.get(),
                                                    0.0,
                                                    DriveConstants.kHeadingKd.get()),
                                    DriveConstants.kHeadingKp,
                                    DriveConstants.kHeadingKd);

                            if (lockWheels.getAsBoolean()) {
                                RobotState.getInstance().setWheelsLocked(true);
                                drive.stopWithX();
                                return;
                            }
                            RobotState.getInstance().setWheelsLocked(false);

                            Translation2d linear =
                                    linearVelocityFromJoysticks(
                                            xSupplier.getAsDouble(), ySupplier.getAsDouble());
                            double omega =
                                    heading.calculate(
                                            drive.getRotation().getRadians(),
                                            targetHeading.get().getRadians());

                            double scale = slowMode.getAsBoolean() ? kSlowModeFactor : 1.0;
                            drive.runVelocity(
                                    toFieldRelative(
                                            drive,
                                            linear.getX()
                                                    * drive.getMaxLinearSpeedMetersPerSec()
                                                    * scale,
                                            linear.getY()
                                                    * drive.getMaxLinearSpeedMetersPerSec()
                                                    * scale,
                                            omega));

                            Logger.recordOutput(
                                    "Drive/HeadingTarget", targetHeading.get().getRadians());
                            Logger.recordOutput("Drive/HeadingAtGoal", heading.atGoal());
                        })
                .beforeStarting(() -> heading.reset(drive.getRotation().getRadians()))
                .withName("Drive at angle");
    }

    /**
     * Field-relative to robot-relative.
     *
     * <p>The alliance flip is applied by rotating the commanded heading by 180 degrees on red,
     * which is what "forward is away from your own wall" means. The previous code used {@code
     * AllianceFlipUtil.apply(Rotation2d)}, which mirrors a rotation about the X axis -- correct for
     * flipping a field <em>position</em>, wrong for converting driver intent, and it made the
     * robot's response to the stick mirror-imaged on red.
     */
    private static ChassisSpeeds toFieldRelative(Drive drive, double vx, double vy, double omega) {
        Rotation2d heading = drive.getRotation();
        if (RobotState.getInstance().isRedAlliance()) {
            heading = heading.plus(Rotation2d.k180deg);
        }
        return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, heading);
    }

    /** Buzz a controller for as long as the command runs, and reliably stop on the way out. */
    public static Command rumble(XboxController controller) {
        return Commands.startEnd(
                        () -> controller.setRumble(RumbleType.kBothRumble, kSlowModeRumble),
                        () -> controller.setRumble(RumbleType.kBothRumble, 0.0))
                .withName("Rumble");
    }

    /**
     * Spin slowly in place and compare wheel rotation against gyro rotation to measure the
     * effective wheel radius.
     *
     * <p>New. The wheel diameter in {@link DriveConstants} carries a hand-tuned fudge factor of
     * 1.613/1.664 with no record of how it was obtained; this is the routine that produces that
     * number, so it can be re-measured in a few seconds whenever the tread is changed rather than
     * guessed at. Run it from the dashboard on a clear patch of carpet.
     */
    public static Command wheelRadiusCharacterization(Drive drive) {
        double[] startPositions = new double[4];
        Rotation2d[] startHeading = new Rotation2d[1];

        return Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    double[] now = drive.getWheelRadiusCharacterizationPositions();
                                    System.arraycopy(now, 0, startPositions, 0, 4);
                                    startHeading[0] = drive.getGyroRotation();
                                }),
                        drive.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.5)))
                                .withTimeout(12.0),
                        Commands.runOnce(
                                () -> {
                                    double[] now = drive.getWheelRadiusCharacterizationPositions();
                                    double wheelDelta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheelDelta += Math.abs(now[i] - startPositions[i]) / 4.0;
                                    }
                                    double gyroDelta =
                                            Math.abs(
                                                    drive.getGyroRotation()
                                                            .minus(startHeading[0])
                                                            .getRadians());
                                    double radius =
                                            (gyroDelta * DriveConstants.kDriveBaseRadiusMeters)
                                                    / wheelDelta;
                                    Logger.recordOutput("Drive/MeasuredWheelRadiusMeters", radius);
                                    System.out.printf(
                                            "Measured wheel radius: %.5f m (%.4f in)%n",
                                            radius, radius / 0.0254);
                                }))
                .withName("Wheel radius characterization");
    }
}
