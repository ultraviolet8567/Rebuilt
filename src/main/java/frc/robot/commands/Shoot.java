package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Swerve;

public class Shoot extends Command {
    private Flywheel flywheel;
    private Odometry odometry;
    private Swerve swerve;

    public Shoot(Flywheel flywheel) {
        this.flywheel = flywheel;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.start();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }
}
