package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.SimBattery;

/** Physics model of the funnel rollers. */
public class FunnelIOSim implements FunnelIO {
    private static final DCMotor kGearbox = DCMotor.getKrakenX60(1);

    private final DCMotorSim sim =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            kGearbox,
                            IntakeConstants.kFunnelSimMOI,
                            IntakeConstants.kFunnelGearing),
                    kGearbox);

    private double volts = 0.0;

    @Override
    public void updateInputs(FunnelIOInputs inputs) {
        sim.setInputVoltage(volts);
        sim.update(Constants.kLoopPeriodSecs);

        inputs.connected = true;
        inputs.velocityRps = sim.getAngularVelocityRPM() / 60.0;
        inputs.appliedVolts = volts;
        inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());

        SimBattery.addCurrent(sim.getCurrentDrawAmps());
    }

    @Override
    public void setVoltage(double v) {
        volts = MathUtil.clamp(v, -IntakeConstants.kFunnelVolts, IntakeConstants.kFunnelVolts);
    }

    @Override
    public void stop() {
        volts = 0.0;
    }
}
