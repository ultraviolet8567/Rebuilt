package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Flywheel;

public class Shoot extends Command {
    private final Flywheel flywheel;
    private final double velocity;

    public Shoot(Flywheel flywheel, double velocity) {
        this.flywheel = flywheel;
        this.velocity = velocity;

        // addRequirements(shooter);
    }

    @Override
    public void initialize() {
        flywheel.start(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }
}
