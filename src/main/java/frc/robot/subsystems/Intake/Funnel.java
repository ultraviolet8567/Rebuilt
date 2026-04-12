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

public class Funnel extends SubsystemBase {

    // Idk what variables 🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🍅

    private final SparkFlex funnelMotor; // Spark Flex Spins
    private final SparkFlexConfig funnelMotorConfig;
    private final RelativeEncoder relativeEncoder;

    public Funnel() {
        System.out.println("[Init] Creating Funnel");

        funnelMotor = new SparkFlex(CAN.kFunnelPort, MotorType.kBrushless);
        funnelMotorConfig = new SparkFlexConfig();
        relativeEncoder = funnelMotor.getEncoder();
        funnelMotorConfig.encoder.velocityConversionFactor(1.0 / IntakeConstants.kFunnelReduction);
        funnelMotorConfig.smartCurrentLimit(80);
        funnelMotor.configure(
                funnelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Intake Constructer Close Bracket

    // public void setRaiseVelocity()

    public void setFunnelVoltage(double voltage) {

        voltage =
                MathUtil.clamp(
                        voltage, -IntakeConstants.kFunnelVoltage, IntakeConstants.kFunnelVoltage);
        voltage *= IntakeConstants.kFunnelInverted ? 1 : -1;
        funnelMotor.setVoltage(voltage);
    }

    // help pls 💔

    public void start() {
        setFunnelVoltage(IntakeConstants.kFunnelVoltage);
    }

    public void stop() {
        setFunnelVoltage(0);
    }
} // DO NOT CODE PAST THIS LINE
