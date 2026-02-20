package frc.robot.subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.Hood.HoodIO.HoodIOInputs;

public interface HoodIO {
    class HoodIOInputs {
    public double velocityRPM = 0.0;
		public double targetVelocityRPM = 0.0;
		public double[] appliedVoltage = new double[]{0.0, 0.0};
		public double[] currentAmps = new double[]{0.0, 0.0};
		public double[] tempCelsius = new double[]{0.0, 0.0};

		public double hoodAbsoluteEncoderValue = 0.0; 
		public double hoodRotations = 0.0;
		public double hoodAngle = 0.0; 
    }
    default void updateInputs(HoodIOInputs inputs) {
	}

	default double getRotationRads() {
		return 0;
	}

	default double getHoodAngleRads() {
		return 0;
	}

    default void setHoodInputVoltage(double volts) {
	}

    default double getHoodAbsoluteRotationRads() {
	return 0;
	}

    default void stopHood() {
	}
    
    default void setHoodRads(double rads) {
	}

	default void resetHoodEncoder() {
	}

}

