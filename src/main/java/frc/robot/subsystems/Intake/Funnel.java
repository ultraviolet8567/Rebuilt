package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;

public class Funnel extends SubsystemBase {

    // Idk what variables 🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🍅

    private final TalonFX funnelMotor; // Spark Flex Spins
    private final TalonFXConfiguration funnelMotorConfig;

    public Funnel() {
        System.out.println("[Init] Creating Funnel");

        funnelMotor = new TalonFX(CAN.kFunnelPort);
        funnelMotorConfig = new TalonFXConfiguration();
        funnelMotorConfig.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(40))
                        .withStatorCurrentLimitEnable(true));
        funnelMotorConfig.withMotorOutput(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));

        funnelMotor.getConfigurator().apply(funnelMotorConfig);
    }

    // Intake Constructer Close Bracket

    // public void setRaiseVelocity()

    public void setFunnelVoltage(double voltage) {

        voltage =
                MathUtil.clamp(
                        voltage, -IntakeConstants.kFunnelVoltage, IntakeConstants.kFunnelVoltage);
        voltage *= IntakeConstants.kFunnelInverted ? -1 : 1;
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
