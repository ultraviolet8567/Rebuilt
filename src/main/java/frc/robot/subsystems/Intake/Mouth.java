package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;

public class Mouth extends SubsystemBase {

    // Idk what variables 🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🍅

    private final SparkFlex mouthMotor; // Spark Flex Spins
    private final SparkFlexConfig mouthMotorConfig;
    private final RelativeEncoder mouthEncoder;

    public Mouth() {
        System.out.println("[Init] Creating Intake");

        mouthMotor = new SparkFlex(CAN.kMouthPort, MotorType.kBrushless);
        mouthMotorConfig = new SparkFlexConfig();
        mouthEncoder = mouthMotor.getEncoder();
        mouthMotorConfig.encoder.velocityConversionFactor(1.0 / IntakeConstants.kMouthReduction);
        mouthMotorConfig.smartCurrentLimit(80);
        mouthMotor.configure(
                mouthMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Intake Constructer Close Bracket

    // public void setRaiseVelocity()

    public void setMouthVoltage(double voltage) {

        voltage =
                MathUtil.clamp(
                        voltage,
                        -IntakeConstants.kMouthMaxVoltage,
                        IntakeConstants.kMouthMaxVoltage);
        voltage *= IntakeConstants.kMouthInverted ? 1 : -1;
        mouthMotor.setVoltage(voltage);
    }

    // help pls 💔

    public void start() {
        setMouthVoltage(IntakeConstants.kMouthVoltage);
    }

    public void stop() {
        setMouthVoltage(0);
    }
} // DO NOT CODE PAST THIS LINE
