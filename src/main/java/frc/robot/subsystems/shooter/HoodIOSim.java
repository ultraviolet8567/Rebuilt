package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.SimBattery;

/**
 * Physics model of the hood.
 *
 * <p>Both encoders are derived from one state variable, so they cannot drift apart the way the
 * previous simulation's could: there, REV's {@code SparkMaxSim} owned the relative position and the
 * physics model owned the absolute one, and re-seeding had to write to both.
 *
 * <p>The absolute reading still aliases exactly as it does on the robot -- the encoder sits before
 * the rack reduction, so it wraps every {@code 2*pi/kHoodRackReduction} radians of hood travel --
 * because that aliasing is what the re-sync window in {@link Hood} exists to tolerate, and a
 * simulation that hid it would not exercise that code.
 */
public class HoodIOSim implements HoodIO {
    private static final DCMotor kGearbox = DCMotor.getNEO(1);

    private final DCMotorSim sim =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            kGearbox,
                            ShooterConstants.kHoodSimMOI,
                            ShooterConstants.kHoodTotalReduction),
                    kGearbox);

    private double volts = 0.0;
    private double encoderOffsetRad;

    public HoodIOSim() {
        sim.setState(ShooterConstants.kHoodLowerRad, 0.0);
        encoderOffsetRad = 0.0;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        // Hard stops: the hood cannot travel past its mechanical limits.
        double angle = sim.getAngularPositionRad();
        if (angle < ShooterConstants.kHoodLowerRad - 0.05) {
            sim.setState(ShooterConstants.kHoodLowerRad - 0.05, 0.0);
        } else if (angle > ShooterConstants.kHoodUpperRad + 0.05) {
            sim.setState(ShooterConstants.kHoodUpperRad + 0.05, 0.0);
        }

        sim.setInputVoltage(volts);
        sim.update(Constants.kLoopPeriodSecs);

        inputs.motorConnected = true;
        inputs.absoluteEncoderConnected = true;
        inputs.relativeAngleRad = sim.getAngularPositionRad() + encoderOffsetRad;
        inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = volts;
        inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
        inputs.absoluteAngleRad = encodeAbsolute(sim.getAngularPositionRad());

        SimBattery.addCurrent(sim.getCurrentDrawAmps());
    }

    /** The angle the absolute encoder would report for this true hood angle, including aliasing. */
    private static double encodeAbsolute(double hoodAngleRad) {
        double zero =
                MathUtil.inputModulus(ShooterConstants.kHoodEncoderOffsetRad, -Math.PI, Math.PI);
        double sign = ShooterConstants.kHoodAbsoluteEncoderInverted ? -1 : 1;
        double rotations =
                MathUtil.inputModulus(
                        sign
                                * (hoodAngleRad - zero)
                                * ShooterConstants.kHoodRackReduction
                                / (2 * Math.PI),
                        0.0,
                        1.0);
        return HoodIOSpark.decodeAbsolute(rotations);
    }

    @Override
    public void setVoltage(double v) {
        volts = MathUtil.clamp(v, -ShooterConstants.kHoodMaxVolts, ShooterConstants.kHoodMaxVolts);
    }

    @Override
    public void seedRelativeEncoder(double angleRad) {
        encoderOffsetRad = angleRad - sim.getAngularPositionRad();
    }

    @Override
    public void stop() {
        volts = 0.0;
    }
}
