package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ManualTeleOp extends Command {
    private final Swerve swerve;
    private final Odometry odometry;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> rightBumper, xButton;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public ManualTeleOp(
            Swerve swerve,
            Odometry odometry,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> rightBumper,
            Supplier<Boolean> xButton) {
        this.swerve = swerve;
        this.odometry = odometry;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.rightBumper = rightBumper;
        this.xButton = xButton;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter =
                new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (xButton.get()) {
            swerve.lockWheels();
            return;
        }

        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        xSpeed *= (xSpeed > 0) ? (1.0 / 0.8) : (1.0 / 0.9);
        ySpeed *= (1.0 / 0.9);

        if (Math.abs(xSpeed) < OIConstants.kDeadband) {
            xSpeed = 0;
        }
        if (Math.abs(ySpeed) < OIConstants.kDeadband) {
            ySpeed = 0;
        }
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0;

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
        turningSpeed = MathUtil.clamp(turningSpeed, -1, 1);

        // double teleMaxSpeed = Lights.getInstance().isDemo
        // ? DriveConstants.kDemoTeleDriveMaxSpeedMetersPerSecond
        // : DriveConstants.kTeleDriveMaxSpeedMetersPersecond;
        // double teleMaxAngularSpeed = Lights.getInstance().isDemo?
        // DriveConstants.kDemoTeleDriveMaxAngularSpeedRadiansPerSecond
        // : DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed =
                turningLimiter.calculate(turningSpeed)
                        * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        Rotation2d currentHeading = odometry.getHeading();
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
