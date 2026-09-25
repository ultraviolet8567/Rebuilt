package frc.robot.subsystems.shooter;

import frc.robot.util.SynthesisDevices;

/** The local kicker model, with its output published so Synthesis knows when fuel is being fed. */
public class KickerIOSynthesis implements KickerIO {
    private final KickerIOSim sim = new KickerIOSim();
    private final SynthesisDevices.Motor output =
            new SynthesisDevices.Motor("Kicker", ShooterConstants.kKickerId);

    @Override
    public void updateInputs(KickerIOInputs inputs) {
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
