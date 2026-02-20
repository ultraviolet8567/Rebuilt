package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
	@AutoLog
	class ShooterIOInputs {
		public double velocityRPM = 0.0;
		public double targetVelocityRPM = 0.0;
		public double[] flywheelAppliedVoltage = new double[]{0.0, 0.0};
		public double[] currentAmps = new double[]{0.0, 0.0};
		public double[] tempCelsius = new double[]{0.0, 0.0};

		public double kickerAppliedVoltage = 0.0;

		public double hoodAppliedVoltage = 0.0; 
		public double hoodAbsoluteEncoderValue = 0.0; 
		public double hoodRotations = 0.0;
		public double hoodAngle = 0.0; 
	}

	default double getFlywheelVelocity() {
		return 0;
	}

	default void updateInputs(ShooterIOInputs inputs) {
	}

	default double getRotationRads() {
		return 0;
	}

	default double getHoodAngleRads() {
		return 0;
	}

	/** Sets motor voltage */
	default void setFlywheelInputVoltage(double volts) {
	}
	default void setKickerInputVoltage(double volts) {
	}
	default void setHoodInputVoltage(double volts) {
	}
	

	/** Stops the motors */
	default void stopFlywheel() {
	}

	default void stopKicker() { 		
	}

	default void stopHood() {
	}

	/** Sets the PID and feed-forward parameters */
	default void setGains(double tkP, double tkI, double tkD, double tffkS, double tffkV, double bkP, double bkI,
			double bkD, double bffkS, double bffkV) {
	}

	/** Sets the idle mode of the motor */
	default void setBrakeMode(boolean brake) {
	}

	/** Run flywheels at exact velocity */
	default void setFlywheelVelocity(double flywheelTargetVel) {
	}

	default void setKickerVelocity(double kickerTargetVel) {
	}

	default void setHoodRads(double rads) {
	}

	default void resetHoodEncoder() {
	}
}