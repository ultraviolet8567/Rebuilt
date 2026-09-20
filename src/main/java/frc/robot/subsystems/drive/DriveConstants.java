package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

/** Everything the drivetrain needs to know about itself. */
public final class DriveConstants {
    private DriveConstants() {}

    // ---------------------------------------------------------------- geometry

    public static final double kTrackWidthMeters = Units.inchesToMeters(21.75);
    public static final double kWheelBaseMeters = Units.inchesToMeters(21.75);

    /**
     * Effective wheel diameter. The 1.613/1.664 factor is an empirical correction measured on
     * carpet: it is the ratio of commanded to actual distance travelled, i.e. it absorbs tread
     * compression and scrub. Re-measure it with a wheel-radius characterisation run whenever the
     * tread is replaced.
     */
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.95) * 1.613 / 1.664;

    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2.0;

    /** Distance from the centre of the robot to any module, metres. Used by PathPlanner. */
    public static final double kDriveBaseRadiusMeters =
            Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0);

    /** Module order everywhere in this code: front-left, front-right, back-left, back-right. */
    public static final Translation2d[] kModuleTranslations =
            new Translation2d[] {
                new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2),
                new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2),
                new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2),
                new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2)
            };

    public static final SwerveDriveKinematics kKinematics =
            new SwerveDriveKinematics(kModuleTranslations);

    // ---------------------------------------------------------------- gearing

    /**
     * Which MAXSwerve drive pinion is installed. Motor rotations per wheel rotation. The old code
     * stored these inverted (wheel-turns-per-motor-turn) and then had to divide by them in four
     * different places, including once inside the simulation where the sign of the mistake was not
     * obvious. Stored the conventional way round, every use is a multiply.
     */
    public static final double kDriveGearRatioR1 = 7.03;

    public static final double kDriveGearRatioR2 = 6.03;
    public static final double kDriveGearRatioR3 = 5.27;

    /** The ratio the robot is currently built with. */
    public static final double kDriveGearRatio = kDriveGearRatioR2;

    public static final double kTurnGearRatio = 26.0;

    /** Spark encoder conversion factors: native rotations/RPM into metres and m/s. */
    public static final double kDrivePositionFactor =
            (Math.PI * kWheelDiameterMeters) / kDriveGearRatio;

    public static final double kDriveVelocityFactor = kDrivePositionFactor / 60.0;

    public static final double kTurnPositionFactor = (2 * Math.PI) / kTurnGearRatio;
    public static final double kTurnVelocityFactor = kTurnPositionFactor / 60.0;

    // ---------------------------------------------------------------- limits

    public static final double kMaxSpeedMetersPerSec = 5.0;
    public static final double kMaxAngularSpeedRadPerSec = 3.5 * Math.PI;

    /**
     * Demo mode scales both limits by the same factor, so the robot handles the same, just slower.
     */
    public static final double kDemoScaleFactor = 0.55;

    public static final int kDriveCurrentLimitAmps = 40;
    public static final int kTurnCurrentLimitAmps = 40;

    // ---------------------------------------------------------------- CAN / DIO

    public static final int[] kDriveMotorIds = {10, 11, 12, 13};
    public static final int[] kTurnMotorIds = {20, 21, 22, 23};
    public static final int[] kAbsoluteEncoderPorts = {0, 1, 2, 3};
    public static final int kPigeonId = 30;

    public static final boolean kDriveInverted = true;
    public static final boolean kTurnInverted = false;
    public static final boolean kAbsoluteEncoderInverted = false;

    /**
     * Absolute-encoder zero offsets, radians, in module order.
     *
     * <p>Each value is the accumulated result of every field recalibration the team has done, most
     * recently at BattleCry. They are applied modulo 2*pi, so values outside [-pi, pi] are normal
     * and harmless.
     *
     * <p>To recalibrate: point all four wheels straight forward, read each absolute encoder's raw
     * voltage from {@code Drive/Module{n}/AbsoluteEncoderVolts} in the log, convert to radians
     * (volts / 5V * 2*pi), and negate.
     */
    public static final double[] kAbsoluteEncoderOffsetsRad = {
        2.15 + 3.05 + 0.01, // front left,  adjusted at BattleCry
        -1.7 - 2.954, // front right
        2.06 + 2.99 + 0.109, // back left,   adjusted at BattleCry
        1.43 + 1.99 - 1.793 - Math.PI // back right
    };

    // ---------------------------------------------------------------- control gains

    /** Steering position loop, volts per radian of error. */
    public static final double kTurnKp = 3.0;

    public static final double kTurnKd = 0.0;

    /**
     * Drive velocity loop. kV is the dominant term: volts per m/s, i.e. 12 V divided by the free
     * speed. kP only has to clean up load disturbances.
     *
     * <p>The old code drove the wheels open loop -- {@code motor.set(speed / kPhysicalMaxSpeed)} --
     * which assumes a 12 V battery and no load. Under a sagging battery, or pushing against another
     * robot, the wheels turned slower than the kinematics assumed, so odometry over-counted
     * distance and auto paths came up short. 36 of 42 corpus robots close the loop on velocity.
     */
    public static final double kDriveKs = 0.1;

    public static final double kDriveKv = 12.0 / kMaxSpeedMetersPerSec;
    public static final double kDriveKp = 0.1;
    public static final double kDriveKd = 0.0;

    /** Heading-hold loop used when aiming at the hub. Radians in, rad/s out. */
    public static final LoggedTunableNumber kHeadingKp =
            new LoggedTunableNumber("Drive/HeadingKp", 7.0);

    public static final LoggedTunableNumber kHeadingKd =
            new LoggedTunableNumber("Drive/HeadingKd", 1.0);
    public static final double kHeadingMaxVelRadPerSec = kMaxAngularSpeedRadPerSec;
    public static final double kHeadingMaxAccelRadPerSecSq = 3 * Math.PI;
    public static final double kHeadingToleranceRad = 0.035;

    // ---------------------------------------------------------------- PathPlanner

    public static final double kRobotMassKg = 50.0;
    public static final double kRobotMOIKgMetersSq = 7.0;
    public static final double kWheelCoefficientOfFriction = 1.0;

    public static final ModuleConfig kModuleConfig =
            new ModuleConfig(
                    kWheelRadiusMeters,
                    kMaxSpeedMetersPerSec,
                    kWheelCoefficientOfFriction,
                    DCMotor.getNeoVortex(1).withReduction(kDriveGearRatio),
                    kDriveCurrentLimitAmps,
                    1);

    public static final RobotConfig kPathPlannerConfig =
            new RobotConfig(kRobotMassKg, kRobotMOIKgMetersSq, kModuleConfig, kModuleTranslations);

    public static final PPHolonomicDriveController kPathFollower =
            new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0, 0), new PIDConstants(5.0, 0, 0));

    // ---------------------------------------------------------------- simulation

    /** Reflected inertia of one wheel plus its share of the chassis, kg*m^2. */
    public static final double kDriveSimMOI = 0.03;

    public static final double kTurnSimMOI = 0.004;
}
