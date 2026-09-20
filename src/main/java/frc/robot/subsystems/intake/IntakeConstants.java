package frc.robot.subsystems.intake;

import frc.robot.util.LoggedTunableNumber;

/** Intake pivot and funnel parameters. */
public final class IntakeConstants {
    private IntakeConstants() {}

    public static final int kPivotMotorId = 5;
    public static final int kPivotEncoderPort = 1;
    public static final int kFunnelMotorId = 6;

    // ---------------------------------------------------------------- pivot

    public static final double kPivotGearboxReduction = 45.0;
    public static final double kPivotChainReduction = 40.0 / 16.0;
    public static final double kPivotTotalReduction = kPivotGearboxReduction * kPivotChainReduction;

    public static final double kPivotMaxVolts = 10.0;
    public static final int kPivotCurrentLimitAmps = 50;
    public static final boolean kPivotInverted = false;
    public static final boolean kPivotAbsoluteEncoderInverted = true;

    /** Absolute-encoder zero offset, radians. Accumulated from field recalibrations. */
    public static final double kPivotEncoderOffsetRad =
            -18.037796326794894 + 0.189 - 0.26 + 0.338 + 0.53 + 0.124 - 0.9161 - 0.194;

    /**
     * Angle added to the pivot angle before it is handed to {@link
     * edu.wpi.first.math.controller.ArmFeedforward}, so that zero means "arm horizontal" -- the
     * frame in which gravity compensation is defined.
     */
    public static final double kPivotGravityOffsetRad =
            2.36 - 4.505 - 1.604 - 1.535 - 1.143 - 2.012 + Math.PI;

    public static final double kPivotToleranceRad = 0.1;
    public static final double kPivotResyncWindowRad = 0.1;
    public static final double kPivotResyncDeadbandRad = 0.01;

    /** Stowed (up). */
    public static final double kPivotStowedRad = 0.1;

    /** Mid-travel, used when handing a ball to the indexer. */
    public static final double kPivotMiddleRad = 0.9;

    /** Deployed (down, collecting). */
    public static final double kPivotDeployedRad = 2.0;

    public static final LoggedTunableNumber kPivotP = new LoggedTunableNumber("Intake/PivotP", 8);
    public static final LoggedTunableNumber kPivotI = new LoggedTunableNumber("Intake/PivotI", 0.0);
    public static final LoggedTunableNumber kPivotD = new LoggedTunableNumber("Intake/PivotD", 0.1);

    public static final LoggedTunableNumber kPivotS = new LoggedTunableNumber("Intake/PivotS", 3);
    public static final LoggedTunableNumber kPivotG = new LoggedTunableNumber("Intake/PivotG", 0);
    public static final LoggedTunableNumber kPivotV = new LoggedTunableNumber("Intake/PivotV", 0);
    public static final LoggedTunableNumber kPivotA = new LoggedTunableNumber("Intake/PivotA", 0.0);

    /**
     * Shake period for the "feed" action that jostles a stuck ball loose, seconds.
     *
     * <p>The old implementation counted loop iterations ({@code count % 50 > 25}), which silently
     * encoded an assumption about the loop period into the behaviour. Expressed in seconds it means
     * the same thing and keeps meaning it if the loop rate ever changes.
     */
    public static final double kFeedShakePeriodSecs = 1.0;

    // ---------------------------------------------------------------- funnel

    /** Motor rotations per funnel rotation. */
    public static final double kFunnelGearing = 3.0;

    public static final double kFunnelVolts = 12.0;
    public static final int kFunnelCurrentLimitAmps = 40;
    public static final boolean kFunnelInverted = true;

    // ---------------------------------------------------------------- simulation

    public static final double kPivotArmLengthMeters = 0.4;
    public static final double kPivotArmMassKg = 4.0;
    public static final double kPivotHardStopMarginRad = 0.05;
    public static final double kFunnelSimMOI = 0.002;
}
