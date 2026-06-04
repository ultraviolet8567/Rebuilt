package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriftTeleOp extends Command {
    private final Swerve swerve;
    private final Odometry odometry;
    private final Supplier<Double> xSpdFunction, ySpdFunction, targetAngle;
    private final Supplier<Boolean> rightBumper, xButton;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final PIDController pidController;

    public DriftTeleOp(
            Swerve swerve,
            Odometry odometry,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> targetAngle,
            Supplier<Boolean> rightBumper,
            Supplier<Boolean> xButton) {
        this.swerve = swerve;
        this.odometry = odometry;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.targetAngle = targetAngle;
        this.rightBumper = rightBumper;
        this.xButton = xButton;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter =
                new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        pidController =
                new PIDController(
                        DriveConstants.kSwerveP.get(),
                        DriveConstants.kSwerveI.get(),
                        DriveConstants.kSwerveD.get());

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        pidController.setP(DriveConstants.kSwerveP.get());
        pidController.setI(DriveConstants.kSwerveI.get());

        pidController.setD(DriveConstants.kSwerveD.get());

        Logger.recordOutput(
                "SwerveTeleOp/CurrentAngle", odometry.getGyrometerHeading().getRadians());

        if (xButton.get()) {
            Lights.getInstance().wheelsLocked = true;
            swerve.lockWheels();
            return;
        }
        Lights.getInstance().wheelsLocked = false;

        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();

        double target = MathUtil.inputModulus(targetAngle.get(), 0, 2 * Math.PI);
        double heading =
                MathUtil.inputModulus(
                        odometry.getHeading().unaryMinus().getRadians(), 0, 2 * Math.PI);

        Logger.recordOutput("SwerveTeleOp/Target", target);
        Logger.recordOutput("SwerveTeleOp/Heading", heading);

        double turningSpeed;

        if (target < heading) {
            if (heading - target < Math.PI) {
                turningSpeed = -pidController.calculate(heading, target);
            } else {
                turningSpeed = -pidController.calculate(heading, target + 2 * Math.PI);
            }
        } else {
            if (target - heading < Math.PI) {
                turningSpeed = -pidController.calculate(heading, target);
            } else {
                turningSpeed = -pidController.calculate(heading, target - 2 * Math.PI);
            }
        }

        xSpeed *= (xSpeed > 0) ? (1.0 / 0.8) : (1.0 / 0.9);
        ySpeed *= (1.0 / 0.9);

        if (Math.abs(xSpeed) < OIConstants.kDeadband) {
            xSpeed = 0;
        }
        if (Math.abs(ySpeed) < OIConstants.kDeadband) {
            ySpeed = 0;
        }

        if (rightBumper.get()) {
            RobotContainer.getDriverJoystick().setRumble(RumbleType.kRightRumble, 0.025);
            xSpeed *= 0.33;
            ySpeed *= 0.33;
            turningSpeed *= 0.33;
        } else {
            RobotContainer.getDriverJoystick().setRumble(RumbleType.kBothRumble, 0);
        }

        xSpeed = MathUtil.clamp(xSpeed, -1, 1);
        ySpeed = MathUtil.clamp(ySpeed, -1, 1);

        // double teleMaxSpeed = Lights.getInstance().isDemo
        // ? DriveConstants.kDemoTeleDriveMaxSpeedMetersPerSecond
        // : DriveConstants.kTeleDriveMaxSpeedMetersPersecond;
        // double teleMaxAngularSpeed = Lights.getInstance().isDemo?
        // DriveConstants.kDemoTeleDriveMaxAngularSpeedRadiansPerSecond
        // : DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        /*
        turningSpeed =
                turningLimiter.calculate(turningSpeed)
                        * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
         */

        turningSpeed =
                MathUtil.clamp(
                        turningSpeed,
                        -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
                        DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);

        Rotation2d currentHeading = odometry.getGyrometerHeading();
        if (DriverStation.getAlliance().get() == Alliance.Red)
            currentHeading = currentHeading.unaryMinus();
        currentHeading = AllianceFlipUtil.apply(currentHeading);

        ChassisSpeeds chassisSpeeds;
        if (Constants.fieldOriented) {
            chassisSpeeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeed, ySpeed, turningSpeed, currentHeading);
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        Logger.recordOutput("SwerveTeleOp/SwerveTeleOp/chassisSpeeds", chassisSpeeds);
        Logger.recordOutput("SwerveTeleOp/SwerveTeleOp/xSpeed", xSpeed);
        Logger.recordOutput("SwerveTeleOp/SwerveTeleOp/ySpeed", ySpeed);
        Logger.recordOutput("SwerveTeleOp/SwerveTeleOp/turningSpeed", turningSpeed);

        swerve.setModuleStates(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }
}
