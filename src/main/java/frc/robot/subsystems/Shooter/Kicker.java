package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
    private final SparkMax kickerMotor;
    private final SparkMaxConfig kickerMotorConfig;
    private final RelativeEncoder kickerEncoder;
    private boolean kickerRunning;

    public Kicker() {
        kickerMotor = new SparkMax(CAN.kKickerPort, MotorType.kBrushless);
        kickerEncoder = kickerMotor.getEncoder();
        kickerMotorConfig = new SparkMaxConfig();
        kickerMotorConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kKickerReduction);
        kickerMotorConfig.smartCurrentLimit(50);
        kickerMotor.configure(
                kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kickerRunning = false;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/Kicker/Voltage", kickerMotor.getAppliedOutput());
        Logger.recordOutput("Shooter/Kicker/KickerRunning", kickerMotor.getAppliedOutput());
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
}
