// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

        public static final double kRobotMass = 25;
        public static final double kRobotMOI = 7.0;
    }
public static final class ArmConstants {
		public static final int kArmEncoderPort = 5;
		public static final double kArmEncoderOffset = -2.65016;
		public static final boolean kArmEncoderReversed = true;

		// Physics
		public static final double kArmLength = 0.58;
		public static final double kArmReduction = 144.0;
		public static final double kArmJKgMetersSquared = 0.515;

		// Constraints
		public static final double kMaxArmAngle = -0.084;
		public static final double kMinArmAngle = -1.76625;

		public static final LoggedTunableNumber kMaxSpeed = new LoggedTunableNumber("Arm/Max Speed", 4.5);
		public static final LoggedTunableNumber kMaxAcceleration = new LoggedTunableNumber("Arm/Max Acceleration", 1);
		public static final LoggedTunableNumber kManualVoltage = new LoggedTunableNumber("Arm/ManualVoltage", 8);

		

		// Control
		public static final LoggedTunableNumber kArmPIDTolerance = new LoggedTunableNumber("Arm/PID Tolerance", 0.0001);
		public static final double kSetpointTolerance = 0.2;

		// Arm characterization
		public static final SysIdRoutine.Config characterizationConfig = new SysIdRoutine.Config(
				Volts.of(2).per(Seconds.of(1)), Volts.of(5), Seconds.of(5));
	}
    public static final class ShooterConstants {

		public static final double shooterDemoScaleFactor = 0.25;

		public static final double kShooterReduction = 1.0;

		public static final double kVelocityThreshold = 0.8;
		public static final double kVelocityThresholdLow = 0.6;

		public static final LoggedTunableNumber kShooterPIDTolerance = new LoggedTunableNumber("Shooter/PID Tolerance",
				0.5);

		// TODO: Change to computed value
		public static final LoggedTunableNumber kAutoShooterExitVel = new LoggedTunableNumber(
				"Auto Shooter Exit Velocity", 10);

		// Constants for auto-aiming
		public static final boolean ampUpperEntry = false;
		public static final boolean speakerUpperEntry = true;

		public static final double ampHoriEntryRange = Math.PI / 6;
		public static final double speakerHoriEntryRange = Math.PI / 2;
                // PID values
                public static final double kP = 0.0;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kS = 0.0;
		public static final double kG = 0.0;
		public static final double kV = 0.0;
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
        
        public static final int kflywheelleadPort = 1;
        public static final int kflywheelfollowerPort = 2;

        public static final int kkickerport = 3;
        
        public static final int khoodport = 4;
    }
    public static final class GainsConstants {
		public static final Gains shooterTopGains {
			new Gains(0.00000065361, 0.0, 0.0, 0.0091151, 0.0018015, 0.0, 0.0);
                        };
		public static final Gains shooterBottomGains{
			new Gains(0.000001136, 0.0, 0.0, 0.06427, 0.0018144, 0.0, 0.0);
		};

	}

	public record Gains(double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {
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
