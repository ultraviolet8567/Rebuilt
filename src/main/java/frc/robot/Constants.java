// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean fieldOriented = true;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class ModuleConstants {
    public static final int kDriveMotorGearing = 2;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.95);
    public static final double kDriveMotorGearR1Ratio = 1 / 7.03;
    public static final double kDriveMotorGearR2Ratio = 1 / 6.03;
    public static final double kDriveMotorGearR3Ratio = 1 / 5.27;
    public static final double kTurningMotorGearRatio = 1 / 26;

    public static final double kDriveEncoderR1Rot2Meter =
        kDriveMotorGearR1Ratio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderR2Rot2Meter =
        kDriveMotorGearR2Ratio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderR3Rot2Meter =
        kDriveMotorGearR3Ratio * Math.PI * kWheelDiameterMeters;

    public static final double kDriveEncoderR1RPM2MeterPerSec = kDriveEncoderR1Rot2Meter / 60;
    public static final double kDriveEncoderR2RPM2MeterPerSec = kDriveEncoderR2Rot2Meter / 60;
    public static final double kDriveEncoderR3RPM2MeterPerSec = kDriveEncoderR3Rot2Meter / 60;

    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    public static final double kPTurning = 0.25;
  }

  // CHANGE LATER:
  public static class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(21.75);
    // Distance between front and back wheels:
    public static final double kWheelBase = Units.inchesToMeters(21.75);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front left (+/+)
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front right (+/-)
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back left (-/+)
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back right (-/-)

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;

    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 1;

    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;

    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
    public static final int kBackRightDriveAbsoluteEncoderPort = 3;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.00;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.00;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.00;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.00;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;
    public static final double kTeleDriveMaxSpeedMetersPerSecond = 4.5;

    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3 * Math.PI;

    public static final double kRobotMass = 25;
    public static final double kRobotMOI = 7.0;
  }

  public static class AutoConstants {
    // Speeds from -1 to 1
    public static final double kAutoXDriveSpeed = 0.0;
    public static final double kAutoYDriveSpeed = 0.5;

    public static final double kAutoTurningSpeed = 0.0;
    public static final double kAutoAlignTolerance = 0.015;

    public static final PPHolonomicDriveController kHolonomicController =
        new PPHolonomicDriveController(new PIDConstants(0.25, 0, 0), new PIDConstants(0.5, 0, 0));
  }

  // CAN = computer area network
  public static class CAN {
    public static final int kFrontLeftDriveMotorPort = 10;
    public static final int kFrontLeftTurningMotorPort = 20;

    public static final int kFrontRightDriveMotorPort = 11;
    public static final int kFrontRightTurningMotorPort = 21;

    public static final int kBackLeftDriveMotorPort = 12;
    public static final int kBackLeftTurningMotorPort = 22;

    public static final int kBackRightDriveMotorPort = 13;
    public static final int kBackRightTurningMotorPort = 23;
  }

  public static class OIConstants {
    public static final ControllerType controllerTypeDriver = ControllerType.XBOX;
    public static final ControllerType controllerTypeOperator = ControllerType.XBOX;

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kDeadband = 0.1;
  }

  public static enum ControllerType {
    XBOX,
    LOGITECH,
    JOYSTICK
  }
}
