package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
    private final SparkMax hoodMotor;
    private final SparkMaxConfig hoodMotorConfig;
    private final RelativeEncoder hoodEncoder;
    private final DutyCycleEncoder absoluteEncoder;
    private final PIDController pidController;

    private double targetPosition;

    public Hood() {
        hoodMotor = new SparkMax(CAN.kHoodPort, MotorType.kBrushless);
        hoodEncoder = hoodMotor.getEncoder();
        hoodMotorConfig = new SparkMaxConfig();
        hoodMotorConfig.encoder.positionConversionFactor(
                1.0
                        / (ShooterConstants.kHoodGearboxReduction
                                * ShooterConstants.kHoodRackReduction));
        hoodMotorConfig.smartCurrentLimit(50);
        hoodMotorConfig.idleMode(IdleMode.kBrake);
        hoodMotor.configure(
                hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        absoluteEncoder = new DutyCycleEncoder(CAN.kHoodEncoderPort);
        pidController =
                new PIDController(
                        ShooterConstants.kHoodP, ShooterConstants.kHoodI, ShooterConstants.kHoodD);

        targetPosition = getAbsoluteRotationRads();

        resetRelativeEncoder();
    }

    public void periodic() {
        Logger.recordOutput("Shooter/Hood/AbsoluteRotation", getAbsoluteRotationRads());
        Logger.recordOutput("Shooter/Hood/RelativeRotation", getRelativeRotationRads());
        Logger.recordOutput("Shooter/Hood/TargetRotation", targetPosition);
        Logger.recordOutput(
                "Shooter/Hood/AppliedVoltage",
                hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage());

        double diff = Math.abs(getAbsoluteRotationRads() - getRelativeRotationRads());
        if (diff > 0.001 && diff < 0.1) resetRelativeEncoder();

        // setAngleRads(targetPosition);
    }

    public double getAbsoluteRotationRads() {
        double angle = absoluteEncoder.get();
        angle *= 2 * Math.PI;
        angle *= ShooterConstants.kHoodAbsoluteEncoderInverted ? -1 : 1;
        angle /= ShooterConstants.kHoodRackReduction;
        angle += ShooterConstants.kHoodEncoderOffset;
        angle = MathUtil.inputModulus(angle, 0, Math.PI * 2);
        return angle;
    }

    public double getRelativeRotationRads() {
        double angle = hoodEncoder.getPosition();
        angle *= 2 * Math.PI * (ShooterConstants.kHoodRelativeEncoderInverted ? -1 : 1);
        return angle;
    }

    public void resetRelativeEncoder() {
        double angle = getAbsoluteRotationRads();
        angle /= 2 * Math.PI;
        angle *= (ShooterConstants.kHoodRelativeEncoderInverted ? -1 : 1);
        hoodEncoder.setPosition(angle);
    }

    public void setAngleRads(double angle) {
        double voltage =
                pidController.calculate(
                        getRelativeRotationRads(),
                        MathUtil.clamp(
                                angle, ShooterConstants.kHoodLower, ShooterConstants.kHoodUpper));
        setHoodVoltage(voltage);
    }

    public void setHoodVoltage(double voltage) {
        voltage =
                MathUtil.clamp(
                        voltage, -ShooterConstants.kHoodVoltage, ShooterConstants.kHoodVoltage);
        voltage *= ShooterConstants.kHoodInverted ? -1 : 1;
        hoodMotor.setVoltage(voltage);
    }

    public void stop() {
        setHoodVoltage(0);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition =
                MathUtil.clamp(
                        targetPosition, ShooterConstants.kHoodLower, ShooterConstants.kHoodUpper);
    }

    public double getTargetPosition() {
        return targetPosition;
    }
}
