package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.util.SimBattery;

/** Independent physics model per shooter side. */
public class FlywheelIOSim implements FlywheelIO {
    private static final DCMotor kGearbox = DCMotor.getNeoVortex(1);

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
        leadSim.setInputVoltage(leadVolts);
        followerSim.setInputVoltage(followerVolts);
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
