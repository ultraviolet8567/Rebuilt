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

public class Intake extends SubsystemBase {

    // Idk what variables 🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🍅
    private final SparkMax intakeRaiseMotor; // Spark Max Raises
    private final SparkMaxConfig intakeRaiseMotorConfig;
    private final RelativeEncoder intakeRaiseEncoder;
    private final SparkFlex intakeSpinMotor; // Spark Flex Spins
    private final SparkFlexConfig intakeSpinMotorConfig;
    private final RelativeEncoder intakeSpinEncoder;

    public Intake() {
        System.out.println("[Init] Creating Intake");
        intakeRaiseMotor = new SparkMax(CAN.kIntakeRaisePort, MotorType.kBrushless);
        intakeRaiseMotorConfig = new SparkMaxConfig();
        intakeSpinMotor = new SparkFlex(CAN.kIntakeSpinPort, MotorType.kBrushless);
        intakeSpinMotorConfig = new SparkFlexConfig();

        intakeRaiseMotorConfig.encoder.velocityConversionFactor(1.0 / IntakeConstants.kIntakeRaiseReduction);
        intakeRaiseMotorConfig.smartCurrentLimit(50);
        intakeRaiseMotor.configure(
                intakeRaiseMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeSpinMotorConfig.encoder.velocityConversionFactor(1.0 / IntakeConstants.kIntakeSpinReduction);        
        intakeSpinMotorConfig.smartCurrentLimit(80);

    } // Intake Constructer Close Bracket

        // public void setRaiseVelocity()
        
        public void setSpinVoltage(double voltage) {
                voltage = MathUtil.clamp(
        voltage, -IntakeConstants.kIntakeSpinVoltage, IntakeConstants.kIntakeSpinVoltage);
                voltage *= IntakeConstants.kSpinInverted ? 1 : -1;
        public void spinStart() 
            kickerMotor.set(voltage);
                }


// help pls 💔


        public void spinStart (double voltage) {
            intakeSpinMotor.setSpinVoltage(IntakeConstants)
        } 
kinTakeIntakeSpinVoltageIntakentkIngtakeSpinVoltage

} // DO NOT CODE PAST THIS LINE

                    