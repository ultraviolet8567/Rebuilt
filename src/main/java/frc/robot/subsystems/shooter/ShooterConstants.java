package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.LoggedTunableNumber;

/** Flywheel, hood and kicker parameters. */
public final class ShooterConstants {
    private ShooterConstants() {}

    // ---------------------------------------------------------------- CAN / DIO

    public static final int kFlywheelLeadId = 1;
    public static final int kFlywheelFollowerId = 2;
    public static final int kKickerId = 3;
    public static final int kHoodMotorId = 4;
    public static final int kHoodEncoderPort = 0;

    // ---------------------------------------------------------------- flywheel

    public static final double kFlywheelReduction = 1.0;
    public static final double kFlywheelMaxVolts = 10.0;
    public static final int kFlywheelCurrentLimitAmps = 80;

    /**
     * True if a positive command must spin the motor backwards.
     *
     * <p>This is now applied by the motor controller's own {@code inverted} setting rather than by
     * negating the voltage at the call site. When the negation happens at the call site the encoder
     * still counts the other way, so the measurement and the command disagree in sign -- which is
     * why the old velocity loop had to read {@code calculate(-getVelocity(...), target)}. Inverting
     * in the configuration flips command and measurement together and the minus sign disappears.
     * Not one of the 42 corpus robots negates at the call site; 37 set it in the config.
     */
    public static final boolean kFlywheelInverted = true;

    /** Velocity the flywheel must be within, RPM, before the kicker will feed. */
    public static final double kFlywheelToleranceRpm = 100.0;

    /** Velocity commanded at boot and by the fixed-speed shot. */
    public static final double kFlywheelDefaultRpm = 2200.0;

    public static final double kDemoScaleFactor = 0.25;

    /**
     * Flywheel velocity loop, volts per RPM of error.
     *
     * <p>Raised from 1e-5. At that value a 100 RPM error produced a 1 millivolt correction: the
     * loop was feedforward only, so whatever offset kV left behind stayed there for good, and --
     * more importantly -- the wheel could not recover the speed it loses when a ball passes through
     * it. The next ball then left at whatever velocity the wheel happened to be at.
     *
     * <p>0.002 V/RPM gives a 0.2 V nudge at the 100 RPM ready-tolerance and about 1 V at 500 RPM,
     * comfortably inside the 10 V clamp. In simulation it brings the steady-state error from 80-115
     * RPM down to under 55 RPM, i.e. inside the tolerance the kicker interlock uses.
     */
    public static final LoggedTunableNumber kFlywheelP =
            new LoggedTunableNumber("Shooter/FlywheelP", 0.002);

    public static final LoggedTunableNumber kFlywheelI =
            new LoggedTunableNumber("Shooter/FlywheelI", 0.0);
    public static final LoggedTunableNumber kFlywheelD =
            new LoggedTunableNumber("Shooter/FlywheelD", 0.0);

    public static final LoggedTunableNumber kLeadS = new LoggedTunableNumber("Shooter/LeadS", 0.0);
    public static final LoggedTunableNumber kLeadV =
            new LoggedTunableNumber("Shooter/LeadV", 0.00183);
    public static final LoggedTunableNumber kLeadA = new LoggedTunableNumber("Shooter/LeadA", 0.0);

    public static final LoggedTunableNumber kFollowerS =
            new LoggedTunableNumber("Shooter/FollowerS", 0.0);
    public static final LoggedTunableNumber kFollowerV =
            new LoggedTunableNumber("Shooter/FollowerV", 0.00185);
    public static final LoggedTunableNumber kFollowerA =
            new LoggedTunableNumber("Shooter/FollowerA", 0.0);

    /** Fixed velocity used by the close-range "shuffle" shot. */
    public static final LoggedTunableNumber kShuffleRpm =
            new LoggedTunableNumber("Shooter/ShuffleRpm", 5200);

    // ---------------------------------------------------------------- range table

    /**
     * Distance to the hub (metres) to flywheel velocity (RPM).
     *
     * <p>This replaces {@code 1310.13 + 1270.33*d - 128.09*d*d}. That quadratic opens downwards: it
     * peaks at 4.96 m and then <em>falls</em>, so it commanded 3926 RPM at 7 m -- less than at 5 m
     * -- and 3275 RPM at 8 m. Every long shot undershot the hub, and the further away the robot
     * was, the worse the undershoot got.
     *
     * <p>The entries below 5 m are the quadratic's own values, so shots inside the range the team
     * actually calibrated are bit-for-bit what they were. Beyond 5 m the table follows the
     * logarithmic fit that was sitting commented out next to the quadratic, offset to join it
     * continuously at 4.5 m; the two fits agree to within 1% everywhere inside the calibrated band,
     * which is good evidence they were fitted to the same data and that the log form is the one
     * that extrapolates sanely.
     *
     * <p>A table also degrades gracefully: {@link InterpolatingDoubleTreeMap} clamps to the nearest
     * entry outside its range instead of running off to nonsense, and new range-test points can be
     * dropped in without refitting anything. 34 of the 42 corpus robots use an interpolating table
     * for exactly this.
     */
    public static final InterpolatingDoubleTreeMap kDistanceToRpm =
            new InterpolatingDoubleTreeMap();

    static {
        kDistanceToRpm.put(1.5, 2927.0);
        kDistanceToRpm.put(2.0, 3338.0);
        kDistanceToRpm.put(2.5, 3685.0);
        kDistanceToRpm.put(3.0, 3968.0);
        kDistanceToRpm.put(3.5, 4187.0);
        kDistanceToRpm.put(4.0, 4342.0);
        kDistanceToRpm.put(4.5, 4433.0);
        // Beyond here the old quadratic turned over. Log fit, joined continuously at 4.5 m.
        kDistanceToRpm.put(5.0, 4590.0);
        kDistanceToRpm.put(5.5, 4732.0);
        kDistanceToRpm.put(6.0, 4861.0);
        kDistanceToRpm.put(7.0, 5091.0);
        kDistanceToRpm.put(8.0, 5290.0);
    }

    /**
     * Distance to the hub (metres) to hood angle (radians).
     *
     * <p>Deliberately left empty. The robot has never been range-tested for hood angle -- the hood
     * is driven to a fixed position and trimmed by the operator -- so there is no data to put here,
     * and inventing numbers would change how the robot shoots on the field. {@link
     * Hood#angleForDistance} falls back to the fixed position while this is empty; add measured
     * (distance, angle) pairs from a range-test session and the fallback stops being used.
     */
    public static final InterpolatingDoubleTreeMap kDistanceToHoodAngle =
            new InterpolatingDoubleTreeMap();

    // ---------------------------------------------------------------- hood

    public static final double kHoodGearboxReduction = 25.0;
    public static final double kHoodRackReduction = 168.0 / 10.0;
    public static final double kHoodTotalReduction = kHoodGearboxReduction * kHoodRackReduction;

    public static final double kHoodLowerRad = 0.01;
    public static final double kHoodUpperRad = 0.3;
    public static final double kHoodMaxVolts = 10.0;
    public static final int kHoodCurrentLimitAmps = 50;

    /**
     * Motor inversion, applied in the controller config.
     *
     * <p>This single flag replaces the old pair {@code kHoodInverted} (negate the command) and
     * {@code kHoodRelativeEncoderInverted} (negate the measurement). They always had to be set to
     * the same value to describe one physical fact -- which way round the motor is bolted in -- and
     * getting one without the other produced a positive-feedback loop that drove the hood into its
     * hard stop.
     */
    public static final boolean kHoodInverted = true;

    public static final boolean kHoodAbsoluteEncoderInverted = false;

    /** Absolute-encoder zero offset, radians. Accumulated from field recalibrations. */
    public static final double kHoodEncoderOffsetRad =
            -0.0992 - 6.276 + 0.001 - 0.024 - 6.264; // = -12.6622, applied modulo 2*pi

    /**
     * How far the absolute and relative encoders may disagree before the relative one is re-seeded,
     * radians. The absolute encoder sits before the rack reduction, so it wraps every
     * 2*pi/kHoodRackReduction of hood travel; outside that window it aliases and must be ignored.
     */
    public static final double kHoodResyncWindowRad = 0.1;

    public static final double kHoodResyncDeadbandRad = 0.001;
    public static final double kHoodToleranceRad = 0.02;

    /** Operator nudge per 20 ms loop while the hood trim button is held, radians. */
    public static final double kHoodTrimStepRad = 0.01;

    public static final LoggedTunableNumber kHoodP = new LoggedTunableNumber("Shooter/HoodP", 40);
    public static final LoggedTunableNumber kHoodI = new LoggedTunableNumber("Shooter/HoodI", 0.0);
    public static final LoggedTunableNumber kHoodD = new LoggedTunableNumber("Shooter/HoodD", 0.0);

    // ---------------------------------------------------------------- kicker

    public static final double kKickerReduction = 3.0;
    public static final double kKickerVolts = 12.0;
    public static final boolean kKickerInverted = true;
    public static final int kKickerCurrentLimitAmps = 50;

    // ---------------------------------------------------------------- simulation

    public static final double kFlywheelSimMOI = 0.004;
    public static final double kHoodSimMOI = 0.002;
    public static final double kKickerSimMOI = 0.001;
}
