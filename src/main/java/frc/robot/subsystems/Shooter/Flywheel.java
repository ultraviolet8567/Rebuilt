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
    private final SparkFlexConfig globalConfig, followerConfig;
    private final RelativeEncoder leadEncoder, followerEncoder;
    private final PIDController pidController;
    private final SimpleMotorFeedforward feedforwardController;

    public Flywheel() {
        leadMotor = new SparkFlex(CAN.kFlywheelLeadPort, MotorType.kBrushless);
        leadEncoder = leadMotor.getEncoder();
        followerMotor = new SparkFlex(CAN.kFlywheelFollowerPort, MotorType.kBrushless);
        followerEncoder = leadMotor.getEncoder();

        globalConfig = new SparkFlexConfig();
        globalConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kFlywheelReduction);
        globalConfig.smartCurrentLimit(80);
        globalConfig.idleMode(IdleMode.kCoast);

        followerConfig = new SparkFlexConfig();
        followerConfig.apply(globalConfig).follow(leadMotor);

        leadMotor.configure(
                globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController =
                new PIDController(
                        ShooterConstants.kFlywheelP.get(),
                        ShooterConstants.kFlywheelI.get(),
                        ShooterConstants.kFlywheelD.get());
        feedforwardController =
                new SimpleMotorFeedforward(
                        ShooterConstants.kFlywheelS.get(), ShooterConstants.kFlywheelV.get());
    }

    public void periodic() {
        Logger.recordOutput("Shooter/Flywheel/Velocity", getVelocity());
        Logger.recordOutput(
                "Shooter/Flywheel/MeasuredVoltage",
                leadMotor.getAppliedOutput() * leadMotor.getBusVoltage());

        pidController.setP(ShooterConstants.kFlywheelP.get());
        pidController.setI(ShooterConstants.kFlywheelI.get());
        pidController.setD(ShooterConstants.kFlywheelD.get());
        feedforwardController.setKs(ShooterConstants.kFlywheelS.get());
        feedforwardController.setKv(ShooterConstants.kFlywheelV.get());
    }

    public double getVelocity() {
        return leadEncoder.getVelocity();
    }

    public void toVelocity(double targetVelocity) {
        double voltage =
                MathUtil.clamp(
                        pidController.calculate(leadEncoder.getVelocity(), targetVelocity)
                                + feedforwardController.calculate(leadEncoder.getVelocity()),
                        -ShooterConstants.kFlywheelVoltage,
                        ShooterConstants.kFlywheelVoltage);
        voltage *= ShooterConstants.kFlywheelInverted ? -1 : 1;

        Logger.recordOutput("Shooter/Flywheel/SetVoltage", voltage);
        leadMotor.setVoltage(voltage);
    }

    public void setFlywheelVoltage(double voltage) {
        voltage =
                MathUtil.clamp(
                        voltage,
                        -ShooterConstants.kFlywheelVoltage,
                        ShooterConstants.kFlywheelVoltage);
        voltage *= ShooterConstants.kFlywheelInverted ? -1 : 1;

        leadMotor.setVoltage(voltage);
    }

    public void stop() {
        leadMotor.setVoltage(0);
    }

    public void start() {
        toVelocity(10);
    }

    public boolean atVelocity(double velocity) {
        return Math.abs(velocity - getVelocity()) < ShooterConstants.kFlywheelVelocityTolerance;
    }
}
