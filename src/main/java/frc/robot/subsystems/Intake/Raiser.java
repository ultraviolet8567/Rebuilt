package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotContainer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;

public class Raiser {
     private final SparkMax intakeRaiseMotor; // Spark Max Raises
    private final SparkMaxConfig intakeRaiseMotorConfig;
    private final RelativeEncoder intakeRaiseEncoder;

     private final PIDController pidController;

     public Raiser() {
        intakeRaiseMotor = new SparkMax(CAN.kIntakeRaisePort, MotorType.kBrushless);
        intakeRaiseMotorConfig = new SparkMaxConfig();

        intakeRaiseMotorConfig.encoder.velocityConversionFactor(1.0 / IntakeConstants.kIntakeRaiseReduction);
        intakeRaiseMotorConfig.smartCurrentLimit(50);
        intakeRaiseMotor.configure(
                intakeRaiseMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        pidController = new PIDController(IntakeConstants.kIntakeRaiseP, IntakeConstants.kIntakeRaiseI, IntakeConstants.kIntakeRaiseD);
     }
}


