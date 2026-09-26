package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.util.SimBattery;

/**
 * Independent physics model per shooter side.
 *
 * <p>Each wheel carries the drag the real shooter showed. The team's measured kV (volts per RPM) is
 * a few percent above an ideal Vortex's, and that difference is friction and windage. Without it
 * the frictionless model ran every setpoint about 2.5% fast -- 105 RPM at a 3.4 m shot -- which put
 * the wheel just outside the 100 RPM ready band, so the kicker interlock never fed again.
 */
public class FlywheelIOSim implements FlywheelIO {
    private static final DCMotor kGearbox = DCMotor.getNeoVortex(1);

    /** Volts per RPM an ideal motor needs; the rest of the measured kV is drag. */
    private static final double kIdealVoltsPerRpm =
            1.0
                    / (kGearbox.KvRadPerSecPerVolt * 60.0 / (2.0 * Math.PI))
                    * ShooterConstants.kFlywheelReduction;

    private final FlywheelSim leadSim = makeSim();
    private final FlywheelSim followerSim = makeSim();

    private double leadVolts = 0.0;
    private double followerVolts = 0.0;

    private static FlywheelSim makeSim() {
        return new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        kGearbox,
                        ShooterConstants.kFlywheelSimMOI,
                        ShooterConstants.kFlywheelReduction),
                kGearbox);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        leadSim.setInputVoltage(leadVolts - drag(ShooterConstants.kLeadV.get(), leadSim));
        followerSim.setInputVoltage(
                followerVolts - drag(ShooterConstants.kFollowerV.get(), followerSim));
        leadSim.update(Constants.kLoopPeriodSecs);
        followerSim.update(Constants.kLoopPeriodSecs);

        inputs.leadConnected = true;
        inputs.followerConnected = true;
        inputs.leadVelocityRpm = leadSim.getAngularVelocityRPM();
        inputs.followerVelocityRpm = followerSim.getAngularVelocityRPM();
        inputs.leadAppliedVolts = leadVolts;
        inputs.followerAppliedVolts = followerVolts;
        inputs.leadCurrentAmps = Math.abs(leadSim.getCurrentDrawAmps());
        inputs.followerCurrentAmps = Math.abs(followerSim.getCurrentDrawAmps());

        SimBattery.addCurrent(leadSim.getCurrentDrawAmps());
        SimBattery.addCurrent(followerSim.getCurrentDrawAmps());
    }

    /** Voltage lost to drag at the wheel's current speed. */
    private static double drag(double measuredVoltsPerRpm, FlywheelSim sim) {
        return Math.max(0.0, measuredVoltsPerRpm - kIdealVoltsPerRpm) * sim.getAngularVelocityRPM();
    }

    @Override
    public void setVoltage(double leadVolts, double followerVolts) {
        this.leadVolts = clamp(leadVolts);
        this.followerVolts = clamp(followerVolts);
    }

    private static double clamp(double volts) {
        return MathUtil.clamp(
                volts, -ShooterConstants.kFlywheelMaxVolts, ShooterConstants.kFlywheelMaxVolts);
    }

    @Override
    public void stop() {
        leadVolts = 0.0;
        followerVolts = 0.0;
    }
}
