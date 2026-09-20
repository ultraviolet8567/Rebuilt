package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.util.SimBattery;

/** Physics model of the kicker wheel. */
public class KickerIOSim implements KickerIO {
    private static final DCMotor kGearbox = DCMotor.getNEO(1);

    private final FlywheelSim sim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            kGearbox,
                            ShooterConstants.kKickerSimMOI,
                            ShooterConstants.kKickerReduction),
                    kGearbox);

    private double volts = 0.0;

    @Override
    public void updateInputs(KickerIOInputs inputs) {
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
        volts = MathUtil.clamp(v, -ShooterConstants.kKickerVolts, ShooterConstants.kKickerVolts);
    }

    @Override
    public void stop() {
        volts = 0.0;
    }
}
