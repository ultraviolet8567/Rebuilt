package frc.robot.subsystems.Kicker;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.CAN;
import frc.robot.Constants.KickerConstants;

import org.littletonrobotics.junction.Logger;

public class KickerIOSparkMax implements KickerIO {
    private final SparkMax kickerMotor;
    private final SparkMaxConfig kickerConfig;
    private final RelativeEncoder kickerEncoder;
    
    public KickerIOSparkMax() {
        	System.out.println("[Init] Creating KickerIOSparkMax");
        kickerMotor = new SparkMax(CAN.kKickerPort, MotorType.kBrushless);
        kickerConfig = new SparkMaxConfig();
		kickerEncoder = kickerMotor.getEncoder();
		kickerConfig.inverted(true);
		kickerConfig.encoder.velocityConversionFactor(1.0 / KickerConstants.kKickerReduction);
        
        kickerConfig.smartCurrentLimit(50);

		kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
	public void updateInputs(KickerIOInputs inputs) {
	
		inputs.velocityRPM = kickerEncoder.getVelocity(); 
		inputs.appliedVoltage = kickerMotor.getAppliedOutput() * kickerMotor.getBusVoltage();
		inputs.currentAmps = kickerMotor.getOutputCurrent();
		inputs.tempCelsius = kickerMotor.getMotorTemperature();
    }

    @Override
	public void setKickerInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		kickerMotor.setVoltage(appliedVolts);
	}
    
    @Override
	public void stopKicker() {
		setKickerInputVoltage(0.0);
	}


    

}


