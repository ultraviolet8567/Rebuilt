package frc.robot.subsystems.intake;

import frc.robot.util.SynthesisDevices;

/** The local funnel model, with its output published so Synthesis knows when the intake is on. */
public class FunnelIOSynthesis implements FunnelIO {
    private final FunnelIOSim sim = new FunnelIOSim();
    private final SynthesisDevices.Motor output =
            new SynthesisDevices.Motor("Funnel", IntakeConstants.kFunnelMotorId);

    @Override
    public void updateInputs(FunnelIOInputs inputs) {
        sim.updateInputs(inputs);
        output.set(inputs.appliedVolts / 12.0);
    }

    @Override
    public void setVoltage(double volts) {
        sim.setVoltage(volts);
    }

    @Override
    public void stop() {
        sim.stop();
    }
}
