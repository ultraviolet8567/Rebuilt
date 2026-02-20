package frc.robot.subsystems.Kicker;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;

public interface KickerIO {
    @AutoLog
	class KickerIOInputs {
		public double velocityRPM = 0.0;
		public double targetVelocityRPM = 0.0;
		public double appliedVoltage = 0.0;
		public double currentAmps = 0.0;
		public double tempCelsius = 0.0;

}
default void setKickerInputVoltage(double volts) {
	}
	default void updateInputs(KickerIOInputs inputs) {
	}

    default void setKickerVelocity(double kickerTargetVel) {
	}

    default void stopKicker() { 		
	}

}

