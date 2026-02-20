package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class DirectShoot extends Command {
	private Shooter shooter;
	private Odometry odometry;
	private Swerve swerve;

	public DirectShoot(Shooter shooter, Swerve swerve, Odometry odometry) {
		this.shooter = shooter;
		this.odometry = odometry;
		this.swerve = swerve;

		// addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.shootVoltage();
	}


	@Override
	public void end(boolean interrupted) {
		shooter.stopFlywheel();
	}
}