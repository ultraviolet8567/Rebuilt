package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;

public class Raiser {
    private final SparkMax raiserMotor; // Spark Max Raises
    private final SparkMaxConfig raiserMotorConfig;
    private final RelativeEncoder raiserMotorEncoder;
    private final DutyCycleEncoder absoluteEncoder;

    private final PIDController pidController;

    public Raiser() {
        raiserMotor = new SparkMax(CAN.kRaiserPort, MotorType.kBrushless);
        raiserMotorConfig = new SparkMaxConfig();
        raiserMotorEncoder = raiserMotor.getEncoder();
        raiserMotorConfig.encoder.positionConversionFactor(
                1.0
                        / (IntakeConstants.kRaiserGearboxReduction
                                * IntakeConstants.kRaiserChainReduction));
        raiserMotorConfig.smartCurrentLimit(50);
        raiserMotor.configure(
                raiserMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        absoluteEncoder = new DutyCycleEncoder(CAN.kRaiserEncoderPort);

        pidController =
                new PIDController(
                        IntakeConstants.kRaiserP,
                        IntakeConstants.kRaiserI,
                        IntakeConstants.kRaiserD);
    }

    public void setRaiserRads(double velocity) {}
}
