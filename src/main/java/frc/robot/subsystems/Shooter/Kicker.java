package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.util.SimBattery;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
    private final SparkMax kickerMotor;
    private final SparkMaxConfig kickerMotorConfig;
    private final RelativeEncoder kickerEncoder;
    private boolean kickerRunning;

    // Desktop simulation only (null on the real robot).
    private final FlywheelSim kickerSim;
    private final SparkMaxSim kickerSparkSim;

    public Kicker() {
        System.out.println("[Init] Creating Kicker");

        kickerMotor = new SparkMax(CAN.kKickerPort, MotorType.kBrushless);
        kickerEncoder = kickerMotor.getEncoder();
        kickerMotorConfig = new SparkMaxConfig();
        kickerMotorConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kKickerReduction);
        kickerMotorConfig.smartCurrentLimit(50);
        kickerMotor.configure(
                kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kickerRunning = false;

        if (RobotBase.isSimulation()) {
            DCMotor gearbox = DCMotor.getNEO(1);
            kickerSim =
                    new FlywheelSim(
                            LinearSystemId.createFlywheelSystem(
                                    gearbox,
                                    SimConstants.kKickerMOI,
                                    ShooterConstants.kKickerReduction),
                            gearbox);
            kickerSparkSim = new SparkMaxSim(kickerMotor, gearbox);
        } else {
            kickerSim = null;
            kickerSparkSim = null;
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
                "Shooter/Kicker/Voltage",
                kickerMotor.getAppliedOutput() * kickerMotor.getBusVoltage());
        Logger.recordOutput("Shooter/Kicker/Velocity", kickerEncoder.getVelocity());
        // Was logging the applied output under this key; it is meant to be the boolean.
        Logger.recordOutput("Shooter/Kicker/KickerRunning", kickerRunning);
    }

    public void setKickerVoltage(double voltage) {
        voltage =
                MathUtil.clamp(
                        voltage, -ShooterConstants.kKickerVoltage, ShooterConstants.kKickerVoltage);
        voltage *= ShooterConstants.kKickerInverted ? -1 : 1;
        kickerMotor.setVoltage(voltage);
    }

    public void start() {
        kickerRunning = true;
        setKickerVoltage(ShooterConstants.kKickerVoltage);
    }

    public void stop() {
        kickerRunning = false;
        setKickerVoltage(0);
    }

    // ------------------------------------------------------------------------------------
    // Desktop simulation
    // ------------------------------------------------------------------------------------

    @Override
    public void simulationPeriodic() {
        double dt = Constants.kLoopPeriodSecs;
        double vbus = RoboRioSim.getVInVoltage();

        kickerSim.setInputVoltage(kickerSparkSim.getAppliedOutput() * vbus);
        kickerSim.update(dt);
        // Encoder velocity factor is 1/kKickerReduction, i.e. it reports output RPM.
        kickerSparkSim.iterate(kickerSim.getAngularVelocityRPM(), vbus, dt);

        SimBattery.addCurrent(kickerSim.getCurrentDrawAmps());
    }
}
