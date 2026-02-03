package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import java.util.Arrays;

// import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveModule frontLeft, frontRight, backLeft, backRight;
  public final double driveGearRatio, driveRot2Meter, driveRPM2MeterPerSec;
  private double throttle;

  public Swerve(int gearing) {
    System.out.println("[Init] Creating Swerve");

    switch (gearing) {
      case (1):
        driveGearRatio = ModuleConstants.kDriveMotorGearR1Ratio;
        driveRot2Meter = ModuleConstants.kDriveEncoderR1Rot2Meter;
        driveRPM2MeterPerSec = ModuleConstants.kDriveEncoderR1RPM2MeterPerSec;
        break;
      case (2):
        driveGearRatio = ModuleConstants.kDriveMotorGearR2Ratio;
        driveRot2Meter = ModuleConstants.kDriveEncoderR2Rot2Meter;
        driveRPM2MeterPerSec = ModuleConstants.kDriveEncoderR2RPM2MeterPerSec;
        break;
      case (3):
        driveGearRatio = ModuleConstants.kDriveMotorGearR3Ratio;
        driveRot2Meter = ModuleConstants.kDriveEncoderR3Rot2Meter;
        driveRPM2MeterPerSec = ModuleConstants.kDriveEncoderR3RPM2MeterPerSec;
        break;
      default:
        driveGearRatio = ModuleConstants.kDriveMotorGearR2Ratio;
        driveRot2Meter = ModuleConstants.kDriveEncoderR2Rot2Meter;
        driveRPM2MeterPerSec = ModuleConstants.kDriveEncoderR2RPM2MeterPerSec;
    }

    frontLeft =
        new SwerveModule(
            CAN.kFrontLeftDriveMotorPort,
            CAN.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            driveGearRatio,
            driveRot2Meter,
            driveRPM2MeterPerSec);

    frontRight =
        new SwerveModule(
            CAN.kFrontRightDriveMotorPort,
            CAN.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            driveGearRatio,
            driveRot2Meter,
            driveRPM2MeterPerSec);

    backLeft =
        new SwerveModule(
            CAN.kBackLeftDriveMotorPort,
            CAN.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            driveGearRatio,
            driveRot2Meter,
            driveRPM2MeterPerSec);

    backRight =
        new SwerveModule(
            CAN.kBackRightDriveMotorPort,
            CAN.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            driveGearRatio,
            driveRot2Meter,
            driveRPM2MeterPerSec);

    throttle = 1;
  }

  public double getDriveGearRatio() {
    return driveGearRatio;
  }

  public double getDriveRot2Meter() {
    return driveRot2Meter;
  }

  public double getDriveRPM2MeterPerSec() {
    return driveRPM2MeterPerSec;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
    };
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // double maxSpeed = Lights.getInstance().isDemo
    // ? DriveConstants.kDemoTeleDriveMaxSpeedMetersPerSecond
    // : DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

    double maxSpeed = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
    frontLeft.setDesiredState(desiredStates[0], throttle);
    frontRight.setDesiredState(desiredStates[1], throttle);
    backLeft.setDesiredState(desiredStates[2], throttle);
    backRight.setDesiredState(desiredStates[3], throttle);
  }

  // Sets the wheels to 45 degree angles so it doesn't move
  public void lockWheels() {
    SwerveModuleState[] locked =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };

    setModuleStates(locked);
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(new SwerveModule[] {frontLeft, frontRight, backLeft, backRight})
        .mapToDouble(SwerveModule::getTurningPosition)
        .toArray();
  }

  public double solveBodyRot(Pose2d robotPose, Translation3d targetPose) {
    return Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX());
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
}
