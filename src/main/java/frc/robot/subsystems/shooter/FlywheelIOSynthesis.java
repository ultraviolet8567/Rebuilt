package frc.robot.subsystems.shooter;

import frc.robot.util.SynthesisDevices;

/**
 * The local flywheel model, with its speed published to Synthesis.
 *
 * <p>Synthesis launches game pieces; it needs to know how fast. The wheel speed goes out on a motor
 * channel as {@code rpm / kRpmScale} because Synthesis reads motor outputs and nothing else.
 */
public class FlywheelIOSynthesis implements FlywheelIO {
    public static final double kRpmScale = 10000.0;

    private final FlywheelIOSim sim = new FlywheelIOSim();
    private final SynthesisDevices.Motor wheelSpeed =
            new SynthesisDevices.Motor("Flywheel", ShooterConstants.kFlywheelLeadId);

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        sim.updateInputs(inputs);
        wheelSpeed.set(inputs.leadVelocityRpm / kRpmScale);
    }

    @Override
    public void setVoltage(double leadVolts, double followerVolts) {
        sim.setVoltage(leadVolts, followerVolts);
    }

    @Override
    public void stop() {
        sim.stop();
    }
}
