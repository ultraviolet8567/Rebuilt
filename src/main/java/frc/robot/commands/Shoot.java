package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class Shoot extends Command {
	private Shooter shooter;
	private Odometry odometry;
	private Swerve swerve;

	public Shoot(Shooter shooter, Swerve swerve, Odometry odometry) {
		this.shooter = shooter;
		this.odometry = odometry;
		this.swerve = swerve;

		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.shoot();
	}

	@Override
	public void execute() {
		if (shooter.atVelocity()) {
			shooter.kickIn();

			// double targetRotZ = swerve.solveBodyRot(odometry.getPose(), FieldConstants.Speaker.centerSpeakerOpening);
			// double targetRotY = arm.solveArmRot(odometry.getPose(), FieldConstants.Speaker.centerSpeakerOpening,
			// 		ShooterConstants.kAutoShooterExitVel.get(), false);

			// Logger.recordOutput("AutoTargeter/RotZ", targetRotZ);
			// Logger.recordOutput("AutoTargeter/RotY", targetRotY);
		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stopKicker();
	}
}