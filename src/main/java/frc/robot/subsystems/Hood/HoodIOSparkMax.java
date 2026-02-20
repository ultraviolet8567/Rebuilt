package frc.robot.subsystems.Hood;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CAN;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;

import org.littletonrobotics.junction.Logger;

public class HoodIOSparkMax implements HoodIO {
    private final SparkMax hoodMotor;
    private final SparkMaxConfig hoodConfig;
    private final DutyCycleEncoder hoodEncoder;
    private final RelativeEncoder hoodMotorEncoder;
    private final PIDController hoodPID;
    private final SimpleMotorFeedforward hoodFF;

public HoodIOSparkMax() {
   
    hoodPID = new PIDController(HoodConstants.kHoodP, HoodConstants.kHoodI, HoodConstants.kHoodD);
	hoodFF = new SimpleMotorFeedforward(HoodConstants.kHoodS, HoodConstants.kHoodG, HoodConstants.kHoodV);
    
     
	hoodMotor = new SparkMax(CAN.kHoodPort, MotorType.kBrushless);
    hoodConfig = new SparkMaxConfig();
	hoodMotorEncoder = hoodMotor.getEncoder();
	hoodConfig.inverted(true);
	hoodConfig.encoder.positionConversionFactor(1.0 / HoodConstants.kHoodMotorReduction * 2 * Math.PI); // converts position to RADIANS
	hoodConfig.encoder.velocityConversionFactor(1.0 / HoodConstants.kHoodMotorReduction / HoodConstants.kHoodGearReduction); // converts velocity to RPM
	hoodEncoder = new DutyCycleEncoder(CAN.kHoodEncoderPort);
     hoodConfig.smartCurrentLimit(50);

	hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

	    resetHoodEncoder();
}
        @Override
	    public void updateInputs(HoodIOInputs inputs) {
	
		inputs.targetVelocityRPM = targetVel; // TODO: Calculate target velocity
		inputs.velocityRPM = hoodMotor.getVelocity(); 
		inputs.appliedVoltage = new double[]{hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage(), 
			hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage()};
		inputs.currentAmps = new double[]{hoodMotor.getOutputCurrent(), hoodMotor.getOutputCurrent()};
		inputs.tempCelsius = new double[]{hoodMotor.getMotorTemperature(), hoodMotor.getMotorTemperature()};

		inputs.hoodAbsoluteEncoderValue = getHoodAbsoluteRotationRads();
		inputs.hoodRotations = hoodMotorEncoder.getPosition(); 
		inputs.hoodAngle = getHoodAngleRads(); 

	}
	
       @Override
	public void setHoodInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		hoodMotor.setVoltage(appliedVolts);
	}

        @Override
	public void stopHood() {
		setHoodInputVoltage(0.0);
	}

    @Override
	public double getHoodAbsoluteRotationRads() {
		double angle = hoodEncoder.get();
		angle *= 2 * Math.PI;
		angle += HoodConstants.kHoodEncoderOffset;
		angle = MathUtil.inputModulus(angle, -Math.PI, Math.PI);

		return angle * (HoodConstants.kHoodEncoderReversed ? -1 : 1);
	}
        @Override
	public double getRotationRads() {
		return hoodMotorEncoder.getPosition();
	}

	    @Override
	public double getHoodAngleRads() {
		double hoodAngle = getRotationRads() * HoodConstants.kHoodGearReduction; 
		return hoodAngle;
	}
        @Override
	// TODO: HOOD PID! DO NOT USE RIGHT NOW!!!
	public void setHoodRads(double rads) {
		// PID computed voltage to move to the given angle
		double pidVolts = hoodPID.calculate(getHoodAngleRads(), rads);
		double ffVolts = hoodFF.calculate(rads, 0); // not using FF for now

		double voltage = MathUtil.clamp(pidVolts, HoodConstants.kHoodVelocityThresholdLow, HoodConstants.kHoodVelocityThreshold);

		Logger.recordOutput("Shooter/Hood/PID Voltage", pidVolts);
		Logger.recordOutput("Shooter/Hood//FF Voltage", ffVolts);

		setHoodInputVoltage(voltage);
	}    

        


}