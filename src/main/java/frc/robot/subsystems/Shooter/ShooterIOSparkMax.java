package frc.robot.subsystems.Shooter;

// import static frc.robot.Constants.GainsConstants.*;

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
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;


public class ShooterIOSparkMax implements ShooterIO {
	private final SparkFlex flywheelLeadMotor, flywheelFollowerMotor;
    private final SparkMax kickerMotor, hoodMotor;
    private final SparkFlexConfig flywheelLeadConfig, flywheelFollowerConfig;
    private final SparkMaxConfig kickerConfig, hoodConfig;
	private final DutyCycleEncoder hoodEncoder;
	private final RelativeEncoder hoodMotorEncoder, flywheelLeadEncoder, flywheelFollowerEncoder, kickerEncoder;
	private final PIDController flywheelPID, hoodPID;
	private SimpleMotorFeedforward flywheelFF, hoodFF;

	private double targetVel;

	public ShooterIOSparkMax() {
		System.out.println("[Init] Creating ShooterIOSparkMax");

		flywheelPID = new PIDController(ShooterConstants.kFlywheelP, ShooterConstants.kFlywheelI, ShooterConstants.kFlywheelD);
		flywheelFF = new SimpleMotorFeedforward(ShooterConstants.kFlywheelS, ShooterConstants.kFlywheelG, ShooterConstants.kFlywheelV);

		hoodPID = new PIDController(ShooterConstants.kHoodP, ShooterConstants.kHoodI, ShooterConstants.kHoodD);
		hoodFF = new SimpleMotorFeedforward(ShooterConstants.kHoodS, ShooterConstants.kHoodG, ShooterConstants.kHoodV);

		flywheelLeadMotor = new SparkFlex(CAN.kFlywheelLeadPort, MotorType.kBrushless);
		flywheelLeadConfig = new SparkFlexConfig();
		flywheelLeadEncoder = flywheelLeadMotor.getEncoder();
		flywheelLeadConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kFlywheelReduction);

		flywheelFollowerMotor = new SparkFlex(CAN.kFlywheelFollowerPort, MotorType.kBrushless);
		flywheelFollowerConfig = new SparkFlexConfig();
		flywheelFollowerEncoder = flywheelFollowerMotor.getEncoder();
		flywheelFollowerConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kFlywheelReduction);	
       
		kickerMotor = new SparkMax(CAN.kKickerPort, MotorType.kBrushless);
        kickerConfig = new SparkMaxConfig();
		kickerEncoder = kickerMotor.getEncoder();
		kickerConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kKickerReduction);
        
		hoodMotor = new SparkMax(CAN.kHoodPort, MotorType.kBrushless);
        hoodConfig = new SparkMaxConfig();
		hoodMotorEncoder = hoodMotor.getEncoder();
		hoodConfig.encoder.positionConversionFactor(1.0 / ShooterConstants.kHoodMotorReduction / ShooterConstants.kHoodGearReduction * 2 * Math.PI); // converts position to RADIANS
		hoodConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kHoodMotorReduction / ShooterConstants.kHoodGearReduction); // converts velocity to RPM
		hoodEncoder = new DutyCycleEncoder(CAN.kHoodEncoderPort);
      
      	flywheelLeadConfig.idleMode(IdleMode.kBrake);
	    flywheelFollowerConfig.idleMode(IdleMode.kBrake);

// Change smart current limit numbers later
		flywheelLeadConfig.smartCurrentLimit(80);
		flywheelFollowerConfig.smartCurrentLimit(80);

		flywheelFollowerConfig.follow(flywheelLeadMotor, false);

        flywheelLeadMotor.configure(flywheelLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		flywheelFollowerMotor.configure(flywheelFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodConfig.smartCurrentLimit(50);

		hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kickerConfig.smartCurrentLimit(50);

		kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		targetVel = 0;

		resetHoodEncoder();
	}

	@Override 
	public double getFlywheelVelocity() {
		return flywheelLeadEncoder.getVelocity();
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
	
		inputs.targetVelocityRPM = targetVel; // TODO: Calculate target velocity
		inputs.velocityRPM = flywheelLeadEncoder.getVelocity(); 
		inputs.appliedVoltage = new double[]{flywheelLeadMotor.getAppliedOutput() * flywheelLeadMotor.getBusVoltage(), 
			flywheelFollowerMotor.getAppliedOutput() * flywheelFollowerMotor.getBusVoltage()};
		inputs.currentAmps = new double[]{flywheelLeadMotor.getOutputCurrent(), flywheelFollowerMotor.getOutputCurrent()};
		inputs.tempCelsius = new double[]{flywheelLeadMotor.getMotorTemperature(), flywheelFollowerMotor.getMotorTemperature()};

		inputs.hoodAbsoluteEncoderValue = getHoodAbsoluteRotationRads();
		inputs.hoodRotations = hoodMotorEncoder.getPosition(); 
		inputs.hoodAngle = getHoodAngleRads(); 
	}

	@Override
	public void setFlywheelInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		flywheelLeadMotor.setVoltage(appliedVolts);
	}
		@Override
	public void setKickerInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		kickerMotor.setVoltage(appliedVolts);
	}
	@Override
	public void setHoodInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		hoodMotor.setVoltage(appliedVolts);
	}
	

	@Override
	public void stopFlywheel() {
		setFlywheelInputVoltage(0.0);
	}
	@Override
	public void stopKicker() {
		setKickerInputVoltage(0.0);
	}
	@Override
	public void stopHood() {
		setHoodInputVoltage(0.0);
	}

	// @Override
	// public void setBrakeMode(boolean brake) {
	// 	flywheelLeadMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
	// 	flywheelFollowerMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
	// }

	public double getHoodAbsoluteRotationRads() {
		double angle = hoodEncoder.get();
		angle *= 2 * Math.PI;
		angle += ShooterConstants.kHoodEncoderOffset;
		angle = MathUtil.inputModulus(angle, -Math.PI, Math.PI);

		return angle * (ShooterConstants.kHoodEncoderReversed ? -1 : 1);
	}

	@Override
	public double getRotationRads() {
		return hoodMotorEncoder.getPosition();
	}

	@Override
	public double getHoodAngleRads() {
		double hoodAngle = getRotationRads(); 
		return hoodAngle;
	}

	@Override
	// TODO: HOOD PID! DO NOT USE RIGHT NOW!!!
	public void setHoodRads(double rads) {
		// PID computed voltage to move to the given angle
		double pidVolts = hoodPID.calculate(getHoodAngleRads(), rads);
		double ffVolts = hoodFF.calculate(rads, 0); // not using FF for now

		double voltage = MathUtil.clamp(pidVolts, ShooterConstants.kVelocityThresholdLow, ShooterConstants.kVelocityThreshold);

		Logger.recordOutput("Shooter/Hood/PID Voltage", pidVolts);
		Logger.recordOutput("Shooter/Hood//FF Voltage", ffVolts);

		setHoodInputVoltage(voltage);
	}

	@Override
	public void setFlywheelVelocity(double flywheelTargetVel) {
		// PID computed voltage to move to the given height
		double pidVolts = flywheelPID.calculate(getFlywheelVelocity(), flywheelTargetVel);
		double ffVolts = flywheelFF.calculate(flywheelTargetVel, 0); //not using FF for now

		// double voltage = MathUtil.clamp(pidVolts, ShooterConstants.kVelocityThresholdLow, ShooterConstants.kVelocityThreshold);
		double voltage = MathUtil.clamp(pidVolts, -ShooterConstants.kFlywheelVoltage, ShooterConstants.kFlywheelVoltage);
				
		Logger.recordOutput("Shooter/Flywheel/PID Voltage", pidVolts);
		Logger.recordOutput("Shooter/Flywheel/FF Voltage", ffVolts);

		setFlywheelInputVoltage(voltage);
	}
	
	@Override
	public void resetHoodEncoder() {
		hoodMotorEncoder.setPosition(getHoodAbsoluteRotationRads());
	}

		//topTarget = flywheelTargetVel;
}