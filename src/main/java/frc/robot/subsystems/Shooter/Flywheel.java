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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
    private final SparkFlex leadMotor, followerMotor;
    private final SparkFlexConfig leadMotorConfig, followerMotorConfig;
    private final RelativeEncoder leadEncoder, followerEncoder;
    private final PIDController pidController;
    private final SimpleMotorFeedforward feedforwardController;

    private double velocity;
    private boolean running;

    public Flywheel() {
        leadMotor = new SparkFlex(CAN.kFlywheelLeadPort, MotorType.kBrushless);
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
                followerMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        followerMotorConfig.follow(CAN.kFlywheelLeadPort);

        pidController =
                new PIDController(
                        ShooterConstants.kFlywheelP,
                        ShooterConstants.kFlywheelI,
                        ShooterConstants.kFlywheelD);
        feedforwardController =
                new SimpleMotorFeedforward(
                        ShooterConstants.kFlywheelS, ShooterConstants.kFlywheelV);
    }

    public void periodic() {
        if (running) {
            double voltage = feedforwardController.calculate(-velocity);
            setVoltage(voltage);
        }

        Logger.recordOutput("Shooter/Flywheel/Velocity", getVelocity());
        Logger.recordOutput("Shooter/Flywheel/TargetVelocity", velocity);
        Logger.recordOutput("Shooter/Flywheel/Voltage", leadMotor.getAppliedOutput());
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
        voltage *= ShooterConstants.kFlywheelInverted ? -1 : 1;
        Logger.recordOutput("Shooter/Flywheel/InternalVoltage", voltage);
        leadMotor.set(voltage);
    }

    public void stop() {
        running = false;
        leadMotor.setVoltage(0);
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public void start() {
        running = true;
    }

    public boolean atVelocity(double velocity) {
        return Math.abs(velocity - getVelocity()) < ShooterConstants.kFlywheelVelocityTolerance;
    }
}
