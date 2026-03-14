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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
    private final SparkMax pivotMotor; // Spark Max Raises
    private final SparkMaxConfig pivotMotorConfig;
    private final RelativeEncoder relativeEncoder;
    private final DutyCycleEncoder absoluteEncoder;

    private final PIDController pidController;
    private final ArmFeedforward feedforwardController;

    private double targetPosition;

    public Pivot() {
        System.out.println("[Init] Creating Pivot");

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

        resetRelativeEncoder();

        targetPosition = getAbsoluteRotationRads();

        LoggedTunableNumber.ifChanged(hashCode(), () -> pidController.setP(IntakeConstants.kPivotP.get()),IntakeConstants.kPivotP);
        LoggedTunableNumber.ifChanged(hashCode(), () -> pidController.setP(IntakeConstants.kPivotI.get()),IntakeConstants.kPivotI);
        LoggedTunableNumber.ifChanged(hashCode(), () -> pidController.setP(IntakeConstants.kPivotD.get()),IntakeConstants.kPivotD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> feedforwardController.setKs(IntakeConstants.kPivotS.get()),IntakeConstants.kPivotS);
        LoggedTunableNumber.ifChanged(hashCode(), () -> feedforwardController.setKg(IntakeConstants.kPivotG.get()),IntakeConstants.kPivotG);
        LoggedTunableNumber.ifChanged(hashCode(), () -> feedforwardController.setKv(IntakeConstants.kPivotV.get()),IntakeConstants.kPivotV);
    }

    public void periodic() {
        Logger.recordOutput("Intake/Pivot/AbsoluteRotation", getAbsoluteRotationRads());
        Logger.recordOutput("Intake/Pivot/RelativeRotation", getRelativeRotationRads());

        /*
        pidController.setP(IntakeConstants.kPivotP.get());
        pidController.setI(IntakeConstants.kPivotP.get());
        pidController.setD(IntakeConstants.kPivotP.get());
        feedforwardController.setKs(IntakeConstants.kPivotS.get());
        feedforwardController.setKg(IntakeConstants.kPivotG.get());
        feedforwardController.setKv(IntakeConstants.kPivotV.get());
         */

        double diff = Math.abs(getAbsoluteRotationRads() - getRelativeRotationRads());
        if (diff > 0.01 && diff < 0.1) resetRelativeEncoder();

        //setAngleRads(targetPosition);
    }

    public double getAbsoluteRotationRads() {
        double angle = absoluteEncoder.get();
        angle *= 2 * Math.PI;
        angle *= IntakeConstants.kPivotAbsoluteEncoderInverted ? -1 : 1;
        angle /= IntakeConstants.kPivotChainReduction;
        angle += IntakeConstants.kPivotEncoderOffset;
        angle = MathUtil.inputModulus(angle, -Math.PI * 2, Math.PI);
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

    public void setAngleRads(double angle) {
        double pidVoltage =
                pidController.calculate(
                        getRelativeRotationRads(),
                        MathUtil.clamp(
                                angle, IntakeConstants.kPivotLower, IntakeConstants.kPivotUpper));
        double ffVoltage =
                feedforwardController.calculate(getRelativeRotationRads() + 2.36 + Math.PI / 2, 0);
        
        Logger.recordOutput("Intake/Pivot/pidVoltage", pidVoltage);
        Logger.recordOutput("Intake/Pivot/ffVoltage", ffVoltage);

        setPivotVoltage(pidVoltage + ffVoltage);
    }

    public void setPivotVoltage(double voltage) {
        voltage =
                MathUtil.clamp(
                        voltage, -IntakeConstants.kPivotVoltage, IntakeConstants.kPivotVoltage);
        voltage *= IntakeConstants.kPivotInverted ? -1 : 1;
        pivotMotor.setVoltage(voltage);
    }

    public void stop() {
        setPivotVoltage(0);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition =
                MathUtil.clamp(
                        targetPosition, IntakeConstants.kPivotLower, IntakeConstants.kPivotUpper);
    }

    public double getTargetPosition() {
        return targetPosition;
    }
}
