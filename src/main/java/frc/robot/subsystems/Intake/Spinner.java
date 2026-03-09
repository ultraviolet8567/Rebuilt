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

public class Spinner extends SubsystemBase {

    // Idk what variables 🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🍅
   
    private final SparkFlex intakeSpinMotor; // Spark Flex Spins
    private final SparkFlexConfig intakeSpinMotorConfig;
    private final RelativeEncoder intakeSpinEncoder;

   

    public Spinner() {
        System.out.println("[Init] Creating Intake");
       
        intakeSpinMotor = new SparkFlex(CAN.kIntakeSpinPort, MotorType.kBrushless);
        intakeSpinMotorConfig = new SparkFlexConfig();
        intakeSpinEncoder = intakeSpinMotor.getEncoder();
            
        }

       
        intakeSpinMotorConfig.encoder.velocityConversionFactor(1.0 / IntakeConstants.kIntakeSpinReduction);        
        intakeSpinMotorConfig.smartCurrentLimit(80);
        intakeSpinMotor.configure(
                intakeSpinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

       

    } // Intake Constructer Close Bracket

        // public void setRaiseVelocity()
        
        public void setSpinVoltage(double voltage) {
                
            voltage = MathUtil.clamp(
        voltage, -IntakeConstants.kIntakeSpinMaxVoltage, IntakeConstants.kIntakeSpinMaxVoltage);
                voltage *= IntakeConstants.kSpinInverted ? 1 : -1;
       intakeSpinMotor.set(voltage);
                }
        
        

// help pls 💔


        public void spinStart (double voltage) {
        setSpinVoltage(IntakeConstants.kIntakeSpinVoltage);
        } 

        public void spinStop() {
            setSpinVoltage(0.0);
        }


        public void setRaiseVelocity (double velocity) {
            
        }

} // DO NOT CODE PAST THIS LINE

                    
