package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;

// import frc.robot.subsystems.Lights;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    

	private final ShooterIO io;
	private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
	/*
	* Initialize all components and one-time logic to be completed on boot-up here
	*/
    public Shooter(ShooterIO io) {
        this.io = io;
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Shooter", inputs);

		Logger.recordOutput("Shooter/TargetVelocity", getTargetVelocity());
		Logger.recordOutput("Shooter/AtVelocity", atVelocity());
	}

	public void shoot() {
		double targetVel = getTargetVelocity();
		io.setFlywheelVelocity(targetVel);
	}

	public void shoot(double scaleDown) {
		double targetVel = scaleDown * getTargetVelocity();
		io.setFlywheelVelocity(targetVel);
	}

	public void shootVoltage() {
		io.setFlywheelInputVoltage(ShooterConstants.kFlywheelVoltage);
	}

	public boolean atVelocity() {
		// Velocity threshold for Amp shots is lower because shot velocity requires less
		// precision
		// double threshold = (getTargetVelocity() < 800)
		// 		? ShooterConstants.kVelocityThresholdLow
		// 		: ShooterConstants.kVelocityThreshold;

		double threshold = ShooterConstants.kVelocityThreshold; 

		return inputs.velocityRPM >= threshold * inputs.targetVelocityRPM;
	}

	public double getTargetVelocity() {
		/* 
		double vel = switch (arm.getArmMode()) {
			case SPEAKERFRONT -> ShooterConstants.kSpeakerFrontRPM.get();
			case SPEAKERANGLE -> ShooterConstants.kSpeakerAngleRPM.get();
			case SPEAKERSTAGE -> ShooterConstants.kSpeakerStageRPM.get();
			case AMP -> ShooterConstants.kAmpRPM.get();
			case TRAP -> ShooterConstants.kTrapRPM.get();
			default -> ShooterConstants.kIdleRPM.get();
		};
		*/
		double vel = ShooterConstants.kShooterVelocityRPM;
		/* 
		if (Lights.getInstance().isDemo && vel >= 800) {
			return ShooterConstants.shooterDemoScaleFactor * vel;
		} else {
		*/
		return vel;
		// }
	}

	public void kickIn() {
		io.setKickerInputVoltage(ShooterConstants.kKickerVoltage.get());
	}

	public void hoodUp() {
		io.setHoodInputVoltage(ShooterConstants.kHoodVoltage.get());
	}

	public void hoodDown() {
		io.setHoodInputVoltage(-1 * ShooterConstants.kHoodVoltage.get());
	}

	public boolean hoodOutOfUpperBound() {
		return io.getHoodAngleRads() > ShooterConstants.kHoodUpperLimit; 
	}

	public boolean hoodOutOfLowerBound() {
		return io.getHoodAngleRads() < ShooterConstants.kHoodLowerLimit;
	}

	public void stopFlywheel() {
		io.stopFlywheel();
	}

	public void stopKicker() {
		io.stopKicker();
	}

	public void stopHood() {
		io.stopHood();
	}

}