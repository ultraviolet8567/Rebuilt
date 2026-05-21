package frc.robot.subsystems.Storage;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.StorageConstants;

public class Indexer extends SubsystemBase {

    // Idk what variables 🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🍅

    private final SparkFlex indexerMotor; // Spark Flex Spins
    private final SparkFlexConfig indexerMotorConfig;
    private final RelativeEncoder indexerEncoder;

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
    }

    // Intake Constructer Close Bracket

    // public void setRaiseVelocity()

    public void setIndexerVoltage(double voltage) {

        voltage =
                MathUtil.clamp(
                        voltage,
                        -StorageConstants.kIndexerVoltage,
                        StorageConstants.kIndexerVoltage);
        voltage *= StorageConstants.kIndexerInverted ? 1 : -1;
        indexerMotor.setVoltage(voltage);
    }

    // help pls 💔

    public void start() {
        setIndexerVoltage(StorageConstants.kIndexerVoltage);
    }

    public void stop() {
        setIndexerVoltage(0.0);
    }
}
/////////////////////////////////////// Grass///////////////////////////////////
