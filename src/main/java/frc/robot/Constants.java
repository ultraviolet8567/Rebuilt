// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

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
    public static final boolean tuningMode = true;

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
        public static final double kTurningMotorGearRatio = 1 / 26.0;

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

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =
                2.15 + 3.05; // -0.645 + PI
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad =
                -1.7 - 2.954; // -1.497
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2.06 + 2.99; // 1.985
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad =
                1.43 + 1.99 - 1.793 - Math.PI; // -1.529 + PI

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = 3.5;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 2.5 * Math.PI;

        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3 * Math.PI;

        public static final double kRobotMass = 48.5;
        public static final double kRobotMOI = 7.0;

        public static final SwerveModuleState[] kLockStates =
                new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                    new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                    new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                    new SwerveModuleState(0, new Rotation2d(-Math.PI / 4))
                };

        public static final ModuleConfig kRobotModuleConfig =
                new ModuleConfig(
                        ModuleConstants.kWheelDiameterMeters / 2,
                        kPhysicalMaxSpeedMetersPerSecond,
                        1, // friction coefficient between wheel and carpet, (unsure so 1.0)
                        DCMotor.getNEO(1),
                        1 / ModuleConstants.kDriveMotorGearR2Ratio,
                        80,
                        1);
        public static final RobotConfig kRobotConfig =
                new RobotConfig(
                        kRobotMass, // mass,
                        kRobotMOI, // moment of inertia (why), kgm^2
                        kRobotModuleConfig, // module config
                        kDriveKinematics
                                .getModules()); // locations of modules relative of robot center
    }

    public static final class ShooterConstants {
        public static final double kFlywheelVelocityTolerance = 300;
        public static final double kFlywheelReduction = 1.0;
        public static final double kFlywheelVoltage = 10;
        public static final boolean kFlywheelInverted = true;
        public static final double kFlywheelMaxVelocity = 2200;

        public static final double kKickerReduction = 3.0;
        public static final double kKickerVoltage = 5;
        public static final boolean kKickerInverted = true;

        public static final double kHoodLower = 0.01;
        public static final double kHoodUpper = 0.35;

        public static final double kHoodGearboxReduction = 25.0;
        public static final double kHoodRackReduction = 168.0 / 10.0;
        public static final double kHoodEncoderOffset = -0.0992 - 6.276 + 0.001 - 0.024 - 6.264;
        public static final double kHoodVoltage = 10;
        public static final boolean kHoodInverted = true;
        public static final boolean kHoodAbsoluteEncoderInverted = false;
        public static final boolean kHoodRelativeEncoderInverted = true;

        public static final LoggedTunableNumber kHoodTestVoltage =
                new LoggedTunableNumber("HoodTestVoltage", 0.1);

        // PID values
        public static final LoggedTunableNumber kFlywheelP =
                new LoggedTunableNumber("FlywheelPidP", 0.0001);
        public static final LoggedTunableNumber kFlywheelI =
                new LoggedTunableNumber("FlywheelPidI", 0.0001);
        public static final LoggedTunableNumber kFlywheelD =
                new LoggedTunableNumber("FlywheelPidD", 0.0);

        public static final LoggedTunableNumber kFlywheelS =
                new LoggedTunableNumber("FlywheelFfS", 0.00);
        public static final LoggedTunableNumber kFlywheelV =
                new LoggedTunableNumber("FlywheelFfV", 0.0018);
        public static final LoggedTunableNumber kFlywheelA =
                new LoggedTunableNumber("FlywheelFfA", 0.0);

        public static final LoggedTunableNumber kFlywheelVelocity =
                new LoggedTunableNumber("TargetVelocity", 2200);

        public static final LoggedTunableNumber kHoodP = new LoggedTunableNumber("HoodPidP", 5);
        public static final LoggedTunableNumber kHoodI = new LoggedTunableNumber("HoodPidI", 0.0);
        public static final LoggedTunableNumber kHoodD = new LoggedTunableNumber("HoodPidD", 0.0);
    }

    public static final class OdometryConstants {
        public static final String kActiveCamera = "nip";

        public static final Translation3d kTranslationOffset =
                new Translation3d(0.0414, 0.2604, 0.4738);
        public static final Rotation3d kRotationOffset = new Rotation3d(0, -30, 0);
    }

    public static final class IntakeConstants {
        public static final double kPivotGearboxReduction = 5.0;
        public static final double kPivotChainReduction = 40.0 / 16.0;
        public static final double kPivotEncoderOffset =
                -2.223 - 6.086 - Math.PI / 2 - 2.968 - 5.72 - 0.009 + 0.507 - 0.041;
        public static final double kPivotFeedforwardOffset =
                2.36 + Math.PI / 2 - 4.505 + Math.PI - 1.604;
        public static final double kPivotVoltage = 10;
        public static final boolean kPivotInverted = false;
        public static final boolean kPivotAbsoluteEncoderInverted = true;
        public static final boolean kPivotRelativeEncoderInverted = false;

        public static final double kPivotLower = 0.2;
        public static final double kPivotMiddle = 1.0;
        public static final double kPivotUpper = 2.20;

        // Software limit
        // public static final double kRaiseUpper = ;
        // public static final double kRaiseLower = ;

        public static final double kFunnelReduction = 1.0;
        public static final double kFunnelVoltage = 6;
        public static final double kFunnelMaxVoltage = 12;
        public static final boolean kFunnelInverted = false;

        public static final LoggedTunableNumber kPivotP = new LoggedTunableNumber("PivotPidP", 3);
        public static final LoggedTunableNumber kPivotI = new LoggedTunableNumber("PivotPidI", 0.0);
        public static final LoggedTunableNumber kPivotD = new LoggedTunableNumber("PivotPidD", 0.3);

        public static final LoggedTunableNumber kPivotS = new LoggedTunableNumber("PivotFfS", 6);
        public static final LoggedTunableNumber kPivotG = new LoggedTunableNumber("PivotFfG", 1.67);
        public static final LoggedTunableNumber kPivotV = new LoggedTunableNumber("PivotFfV", 1);
        public static final LoggedTunableNumber kPivotA = new LoggedTunableNumber("PivotFfA", 0.0);
    }

    public static final class StorageConstants {
        // change later
        public static final double kIndexerMaxVoltage = 3;
        public static final double kIndexerReduction = 1.0;
        public static final boolean kIndexerInverted = true;
    }

    public static class AutoConstants {
        // Speeds from -1 to 1
        public static final double kAutoXDriveSpeed = 0.0;
        public static final double kAutoYDriveSpeed = 0.5;

        public static final double kAutoTurningSpeed = 0.0;
        public static final double kAutoAlignTolerance = 0.015;

        public static final PPHolonomicDriveController kHolonomicController =
                new PPHolonomicDriveController(
                        new PIDConstants(0.25, 0, 0), new PIDConstants(0.5, 0, 0));
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

        public static final int kFlywheelLeadPort = 1;
        public static final int kFlywheelFollowerPort = 2;

        public static final int kKickerPort = 3;
        public static final int kHoodPort = 4;
        public static final int kHoodEncoderPort = 0;

        public static final int kPivotPort = 5;
        public static final int kPivotEncoderPort = 1;

        public static final int kFunnelPort = 6;

        public static final int kIndexerPort = 7;
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
