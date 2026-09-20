package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.util.SimBattery;

/**
 * Gravity-loaded arm model of the intake pivot.
 *
 * <p>The arm sim works in a frame where zero is horizontal, which is the same frame the team's
 * {@link edu.wpi.first.math.controller.ArmFeedforward} usage assumes -- hence the shared {@code
 * kPivotGravityOffsetRad}. Keeping one conversion pair ({@link #toSimAngle}/{@link #toPivotAngle})
 * instead of scattering the offset means the gravity term the controller computes and the gravity
 * the simulation applies are guaranteed to be describing the same arm.
 */
public class PivotIOSim implements PivotIO {
    private static final DCMotor kGearbox = DCMotor.getNEO(1);

    private final SingleJointedArmSim sim;
    private double volts = 0.0;
    private double encoderOffsetRad = 0.0;

    public PivotIOSim() {
        sim =
                new SingleJointedArmSim(
                        kGearbox,
                        IntakeConstants.kPivotTotalReduction,
                        SingleJointedArmSim.estimateMOI(
                                IntakeConstants.kPivotArmLengthMeters,
                                IntakeConstants.kPivotArmMassKg),
                        IntakeConstants.kPivotArmLengthMeters,
                        toSimAngle(
                                IntakeConstants.kPivotStowedRad
                                        - IntakeConstants.kPivotHardStopMarginRad),
                        toSimAngle(
                                IntakeConstants.kPivotDeployedRad
                                        + IntakeConstants.kPivotHardStopMarginRad),
                        true,
                        toSimAngle(IntakeConstants.kPivotStowedRad));
    }

    static double toSimAngle(double pivotAngleRad) {
        return pivotAngleRad + IntakeConstants.kPivotGravityOffsetRad;
    }

    static double toPivotAngle(double simAngleRad) {
        return simAngleRad - IntakeConstants.kPivotGravityOffsetRad;
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        sim.setInputVoltage(volts);
        sim.update(Constants.kLoopPeriodSecs);

        double pivotAngle = toPivotAngle(sim.getAngleRads());

        inputs.motorConnected = true;
        inputs.absoluteEncoderConnected = true;
        inputs.relativeAngleRad = pivotAngle + encoderOffsetRad;
        inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
        inputs.appliedVolts = volts;
        inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
        inputs.absoluteAngleRad = encodeAbsolute(pivotAngle);

        SimBattery.addCurrent(sim.getCurrentDrawAmps());
    }

    /** What the absolute encoder would report for this true pivot angle, aliasing included. */
    private static double encodeAbsolute(double pivotAngleRad) {
        double zero =
                MathUtil.inputModulus(
                        IntakeConstants.kPivotEncoderOffsetRad, -2 * Math.PI, Math.PI);
        double sign = IntakeConstants.kPivotAbsoluteEncoderInverted ? -1 : 1;
        double rotations =
                MathUtil.inputModulus(
                        sign
                                * (pivotAngleRad - zero)
                                * IntakeConstants.kPivotChainReduction
                                / (2 * Math.PI),
                        0.0,
                        1.0);
        return PivotIOSpark.decodeAbsolute(rotations);
    }

    @Override
    public void setVoltage(double v) {
        volts = MathUtil.clamp(v, -IntakeConstants.kPivotMaxVolts, IntakeConstants.kPivotMaxVolts);
    }

    @Override
    public void seedRelativeEncoder(double angleRad) {
        encoderOffsetRad = angleRad - toPivotAngle(sim.getAngleRads());
    }

    @Override
    public void stop() {
        volts = 0.0;
    }
}
