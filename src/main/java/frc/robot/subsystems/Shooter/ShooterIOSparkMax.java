package frc.robot.subsystems.Shooter;

import static frc.robot.Constants.GainsConstants.*;

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
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;


public class ShooterIOSparkMax implements ShooterIO {
	private final SparkFlex flywheelleadMotor, flywheelfollowerMotor;
    private final SparkMax kickerMotor, hoodMotor;
    private final SparkFlexConfig flywheelleadConfig, flywheelfollowerConfig;
     private final SparkMaxConfig kickerConfig, hoodConfig;
	private final RelativeEncoder hoodEncoder;
	private final PIDController shooterTopPID, shooterBottomPID;
	private SimpleMotorFeedforward shooterTopFF, shooterBottomFF;

	private double topTarget, bottomTarget;

	public ShooterIOSparkMax() {
		System.out.println("[Init] Creating ShooterIOSparkMax");

		flywheelleadMotor = new SparkFlex(CAN.kflywheelleadPort, MotorType.kBrushless);
		flywheelleadConfig = new SparkFlexConfig();
		flywheelleadMotor.setInverted(false);

		flywheelfollowerMotor = new SparkFlex(CAN.kflywheelfollowerPort, MotorType.kBrushless);
		flywheelfollowerConfig = new SparkFlexConfig();
		flywheelfollowerMotor.setInverted(false);

        kickerMotor = new SparkMax(CAN.kkickerport, MotorType.kBrushless);
        kickerConfig = new SparkMaxConfig();

        hoodMotor = new SparkMax(CAN.khoodport, MotorType.kBrushless);
        hoodConfig = new SparkMaxConfig();
      
      flywheelleadConfig.idleMode(IdleMode.kBrake);
	    flywheelfollowerConfig.idleMode(IdleMode.kBrake);
// Change smart current limit numbers later
		 flywheelleadConfig.smartCurrentLimit(80);
		 flywheelfollowerConfig.smartCurrentLimit(80);

		 flywheelfollowerConfig.inverted(false);
		 flywheelfollowerConfig.follow(flywheelleadMotor);

        flywheelleadMotor.configure(flywheelleadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		flywheelfollowerMotor.configure(flywheelfollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

         hoodConfig.smartCurrentLimit(50);

		hoodMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kickerConfig.smartCurrentLimit(50);

		kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


		hoodEncoder = hoodMotor.getEncoder();
		hoodConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kShooterReduction);


		flywheelleadPID = flywheelleadMotor.getPIDController();
		flywheelfollowerPID = flywheelfollowerMotor.getPIDController();

		setGains(shooterTopGains.kP(), shooterTopGains.kI(), shooterTopGains.kD(), shooterTopGains.ffkS(),
				shooterTopGains.ffkV(), shooterBottomGains.kP(), shooterBottomGains.kI(), shooterBottomGains.kD(),
				shooterBottomGains.ffkS(), shooterBottomGains.ffkV());

		topTarget = 0;
		bottomTarget = 0;
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.topVelocityRPM = shooterTopEncoder.getVelocity();
		inputs.topTargetVelocityRPM = topTarget;
		inputs.topAppliedVoltage = shooterTopMotor.getAppliedOutput() * shooterTopMotor.getBusVoltage();
		inputs.topCurrentAmps = new double[]{shooterTopMotor.getOutputCurrent()};
		inputs.topTempCelsius = new double[]{shooterBottomMotor.getMotorTemperature()};

		inputs.bottomVelocityRPM = shooterBottomEncoder.getVelocity();
		inputs.bottomTargetVelocityRPM = bottomTarget;
		inputs.bottomAppliedVoltage = shooterBottomMotor.getAppliedOutput() * shooterBottomMotor.getBusVoltage();
		inputs.bottomCurrentAmps = new double[]{shooterBottomMotor.getOutputCurrent()};
		inputs.bottomTempCelsius = new double[]{shooterBottomMotor.getMotorTemperature()};

		inputs.topRotations = shooterTopEncoder.getPosition();
		inputs.bottomRotations = shooterBottomEncoder.getPosition();
	}

	@Override
	public void setTopInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		flywheelleadMotor.setVoltage(appliedVolts);
	}


	@Override
	public void setInputVoltage(double topVolts, double bottomVolts) {
		setTopInputVoltage(topVolts);
	
	}

	@Override
	public void stop() {
		setInputVoltage(0.0, 0.0);
	}

	@Override
	public void setGains(double tkP, double tkI, double tkD, double tffkS, double tffkV, double bkP, double bkI,
			double bkD, double bffkS, double bffkV) {
		shooterTopPID.setP(tkP);
		shooterTopPID.setI(tkI);
		shooterTopPID.setD(tkI);
		shooterTopFF = new SimpleMotorFeedforward(tffkS, tffkV);

		shooterBottomPID.setP(bkP);
		shooterBottomPID.setI(bkI);
		shooterBottomPID.setD(bkD);
		shooterBottomFF = new SimpleMotorFeedforward(bffkS, bffkV);
	}

	@Override
	public void setBrakeMode(boolean brake) {
		flywheelleadMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
		flywheelfollowerMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public void setVelocity(double topTargetVel, double bottomTargetVel) {
		shooterTopPID.setReference(topTargetVel, SparkBase.ControlType.kVelocity, 0,
				shooterTopFF.calculate(topTargetVel));
		shooterBottomPID.setReference(bottomTargetVel, SparkBase.ControlType.kVelocity, 0,
				shooterBottomFF.calculate(bottomTargetVel));

		topTarget = topTargetVel;
		bottomTarget = bottomTargetVel;
	}
}