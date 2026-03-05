package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Swerve;

public class HoodUp extends Command {
    private Shooter shooter;
    private Odometry odometry;
    private Swerve swerve;

    public HoodUp(Shooter shooter, Swerve swerve, Odometry odometry) {
        this.shooter = shooter;
        this.odometry = odometry;
        this.swerve = swerve;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.getHood().setAngleRads(ShooterConstants.kHoodUpper);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.getHood().stop();
    }
}
