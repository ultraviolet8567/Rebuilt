package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Flywheel;

public class Shoot extends Command {
    private Flywheel flywheel;

    public Shoot(Flywheel flywheel) {
        this.flywheel = flywheel;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.start();
        flywheel.setVelocity(-1000);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }
}
