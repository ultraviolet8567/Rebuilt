package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Pivot {
    private final SparkMax pivotMotor; // Spark Max Raises
    private final SparkMaxConfig pivotMotorConfig;
    private final RelativeEncoder relativeEncoder;
    private final DutyCycleEncoder absoluteEncoder;

    private final PIDController pidController;
    private final ArmFeedforward feedforwardController;

    public Pivot() {
        pivotMotor = new SparkMax(CAN.kPivotPort, MotorType.kBrushless);
        pivotMotorConfig = new SparkMaxConfig();
        relativeEncoder = pivotMotor.getEncoder();
        pivotMotorConfig.encoder.positionConversionFactor(
                1.0
                        / (IntakeConstants.kPivotGearboxReduction
                                * IntakeConstants.kPivotChainReduction));
        pivotMotorConfig.smartCurrentLimit(50);
        pivotMotor.configure(
                pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        absoluteEncoder = new DutyCycleEncoder(CAN.kPivotEncoderPort);

        pidController =
                new PIDController(
                        IntakeConstants.kPivotP.get(),
                        IntakeConstants.kPivotI.get(),
                        IntakeConstants.kPivotD.get());
        feedforwardController =
                new ArmFeedforward(
                        IntakeConstants.kPivotS.get(),
                        IntakeConstants.kPivotG.get(),
                        IntakeConstants.kPivotV.get());
    }

    public double getAbsoluteRotationRads() {
        double angle = absoluteEncoder.get();
        angle *= 2 * Math.PI;
        angle *= IntakeConstants.kPivotAbsoluteEncoderInverted ? -1 : 1;
        angle /= IntakeConstants.kPivotChainReduction;
        angle += IntakeConstants.kPivotEncoderOffset;
        angle = MathUtil.inputModulus(angle, 0, Math.PI * 2);
        return angle;
    }

    public double getRelativeRotationRads() {
        double angle = relativeEncoder.getPosition();
        angle *= 2 * Math.PI * (IntakeConstants.kPivotRelativeEncoderInverted ? -1 : 1);
        return angle;
    }

    public void resetRelativeEncoder() {
        double angle = getAbsoluteRotationRads();
        angle /= 2 * Math.PI;
        angle *= (IntakeConstants.kPivotRelativeEncoderInverted ? -1 : 1);
        relativeEncoder.setPosition(angle);
    }

    public void setPivotRads(double angle) {
        double voltage =
                pidController.calculate(
                        getRelativeRotationRads(),
                        MathUtil.clamp(
                                angle, ShooterConstants.kHoodLower, ShooterConstants.kHoodUpper))+feedforwardController.calculate(angle, 0);
        setPivotVoltage(voltage);
    }

    public void setPivotVoltage(double voltage) {
        voltage =
                MathUtil.clamp(
                        voltage, -ShooterConstants.kHoodVoltage, ShooterConstants.kHoodVoltage);
        voltage *= ShooterConstants.kHoodInverted ? -1 : 1;
        pivotMotor.setVoltage(voltage);
    }

    public void stop() {
        setPivotVoltage(0);
    }
}
