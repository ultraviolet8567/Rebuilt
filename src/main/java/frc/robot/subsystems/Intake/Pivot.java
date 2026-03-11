package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;

public class Pivot {
    private final SparkMax pivotMotor; // Spark Max Raises
    private final SparkMaxConfig pivotMotorConfig;
    private final RelativeEncoder relativeEncoder;
    private final DutyCycleEncoder absoluteEncoder;

    private final PIDController pidController;

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
                        IntakeConstants.kPivotP, IntakeConstants.kPivotI, IntakeConstants.kPivotD);
    }

    public double getAbsoluteRotationRads() {
        double angle = absoluteEncoder.get();
        angle *= 2 * Math.PI * 1.0 / IntakeConstants.kPivotGearboxReduction;
        angle += IntakeConstants.kPivotEncoderOffset;
        angle = MathUtil.inputModulus(angle, 0, Math.PI * 2);
        return angle;
    }

    public double getRelativeRotationRads() {
        double angle = relativeEncoder.getPosition();
        angle *= 2 * Math.PI;
        return angle;
    }

    public void resetRelativeEncoder() {
        relativeEncoder.setPosition(absoluteEncoder.get());
    }

    public void setPivotRads(double velocity) {}
}
