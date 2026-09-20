package frc.robot.subsystems.storage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.util.SimBattery;

/** Physics model of the indexer. */
public class IndexerIOSim implements IndexerIO {
    private static final DCMotor kGearbox = DCMotor.getNeoVortex(1);

    private final FlywheelSim sim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            kGearbox,
                            StorageConstants.kIndexerSimMOI,
                            StorageConstants.kIndexerReduction),
                    kGearbox);

    private double volts = 0.0;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        sim.setInputVoltage(volts);
        sim.update(Constants.kLoopPeriodSecs);

        inputs.connected = true;
        inputs.velocityRpm = sim.getAngularVelocityRPM();
        inputs.appliedVolts = volts;
        inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());

        SimBattery.addCurrent(sim.getCurrentDrawAmps());
    }

    @Override
    public void setVoltage(double v) {
        volts = MathUtil.clamp(v, -StorageConstants.kIndexerVolts, StorageConstants.kIndexerVolts);
    }

    @Override
    public void stop() {
        volts = 0.0;
    }
}
