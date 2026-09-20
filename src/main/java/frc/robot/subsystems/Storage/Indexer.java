package frc.robot.subsystems.Storage;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.StorageConstants;
import frc.robot.util.SimBattery;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final SparkFlex indexerMotor; // Spark Flex Spins
    private final SparkFlexConfig indexerMotorConfig;
    private final RelativeEncoder indexerEncoder;

    // Desktop simulation only (null on the real robot).
    private final FlywheelSim indexerSim;
    private final SparkFlexSim indexerSparkSim;

    public Indexer() {
        System.out.println("[Init] Creating Indexer");

        indexerMotor = new SparkFlex(CAN.kIndexerPort, MotorType.kBrushless);
        indexerMotorConfig = new SparkFlexConfig();
        indexerEncoder = indexerMotor.getEncoder();
        indexerMotorConfig.encoder.velocityConversionFactor(
                1.0 / StorageConstants.kIndexerReduction);
        indexerMotorConfig.smartCurrentLimit(80);
        indexerMotor.configure(
                indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isSimulation()) {
            DCMotor gearbox = DCMotor.getNeoVortex(1);
            indexerSim =
                    new FlywheelSim(
                            LinearSystemId.createFlywheelSystem(
                                    gearbox,
                                    SimConstants.kIndexerMOI,
                                    StorageConstants.kIndexerReduction),
                            gearbox);
            indexerSparkSim = new SparkFlexSim(indexerMotor, gearbox);
        } else {
            indexerSim = null;
            indexerSparkSim = null;
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
                "Storage/Indexer/Voltage",
                indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage());
        Logger.recordOutput("Storage/Indexer/Velocity", indexerEncoder.getVelocity());
    }

    public void setIndexerVoltage(double voltage) {
        voltage =
                MathUtil.clamp(
                        voltage,
                        -StorageConstants.kIndexerVoltage,
                        StorageConstants.kIndexerVoltage);
        voltage *= StorageConstants.kIndexerInverted ? -1 : 1;
        indexerMotor.setVoltage(voltage);
    }

    public void start() {
        setIndexerVoltage(StorageConstants.kIndexerVoltage);
    }

    public void stop() {
        setIndexerVoltage(0.0);
    }

    // ------------------------------------------------------------------------------------
    // Desktop simulation
    // ------------------------------------------------------------------------------------

    @Override
    public void simulationPeriodic() {
        double dt = Constants.kLoopPeriodSecs;
        double vbus = RoboRioSim.getVInVoltage();

        indexerSim.setInputVoltage(indexerSparkSim.getAppliedOutput() * vbus);
        indexerSim.update(dt);
        indexerSparkSim.iterate(indexerSim.getAngularVelocityRPM(), vbus, dt);

        SimBattery.addCurrent(indexerSim.getCurrentDrawAmps());
    }
}
