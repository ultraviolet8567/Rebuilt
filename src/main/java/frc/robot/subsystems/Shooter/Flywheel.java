package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;

public class Flywheel extends SubsystemBase {
    private final SparkFlex leadMotor, followerMotor;
    private final SparkFlexConfig leadMotorConfig, followerMotorConfig;
    private final RelativeEncoder leadEncoder, followerEncoder;

    private final PIDController pidController;

    public Flywheel() {
        leadMotor = new SparkFlex(CAN.kKickerPort, MotorType.kBrushless);
        leadEncoder = leadMotor.getEncoder();
        leadMotorConfig = new SparkFlexConfig();
        leadMotorConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kFlywheelReduction);
        leadMotorConfig.smartCurrentLimit(80);
        leadMotorConfig.idleMode(IdleMode.kCoast);
        leadMotor.configure(
                leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        followerMotor = new SparkFlex(CAN.kFlywheelFollowerPort, MotorType.kBrushless);
        followerEncoder = leadMotor.getEncoder();
        followerMotorConfig = new SparkFlexConfig();
        followerMotorConfig.encoder.velocityConversionFactor(
                1.0 / ShooterConstants.kFlywheelReduction);
        followerMotorConfig.smartCurrentLimit(80);
        followerMotorConfig.idleMode(IdleMode.kCoast);
        followerMotor.configure(
                leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        followerMotorConfig.follow(CAN.kFlywheelLeadPort);

        pidController =
                new PIDController(
                        ShooterConstants.kFlywheelP,
                        ShooterConstants.kFlywheelI,
                        ShooterConstants.kFlywheelD);
    }

    public double getVelocity() {
        return leadEncoder.getVelocity();
    }

    public void setVoltage(double voltage) {
        voltage =
                MathUtil.clamp(
                        voltage,
                        -ShooterConstants.kFlywheelVoltage,
                        ShooterConstants.kFlywheelVoltage);
        voltage *= ShooterConstants.kFlywheelInverted ? 1 : -1;
        leadMotor.set(voltage);
    }

    public void stop() {
        leadMotor.setVoltage(0);
    }

    public void setVelocity(double velocity) {
        double voltage = pidController.calculate(getVelocity(), velocity);
        voltage =
                MathUtil.clamp(
                        voltage,
                        -ShooterConstants.kFlywheelVoltage,
                        ShooterConstants.kFlywheelVoltage);
        setVoltage(voltage);
    }

    public boolean atVelocity(double velocity) {
        return Math.abs(velocity - getVelocity()) < ShooterConstants.kFlywheelVelocityTolerance;
    }
}
