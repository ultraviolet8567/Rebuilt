package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;

public class Kicker extends SubsystemBase {
    private final SparkMax kickerMotor;
    private final SparkMaxConfig kickerMotorConfig;
    private final RelativeEncoder kickerEncoder;

    public Kicker() {
        kickerMotor = new SparkMax(CAN.kKickerPort, MotorType.kBrushless);
        kickerEncoder = kickerMotor.getEncoder();
        kickerMotorConfig = new SparkMaxConfig();
        kickerMotorConfig.inverted(ShooterConstants.kKickerInverted);
		kickerMotorConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kKickerReduction);
        kickerMotorConfig.smartCurrentLimit(50);
        kickerMotor.configure(kickerMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVoltage(double voltage) {
        kickerMotor.set(voltage);
    }

    public void stop() {
        kickerMotor.setVoltage(0);
    }
}

