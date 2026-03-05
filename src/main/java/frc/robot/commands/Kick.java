package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Kicker;

public class Kick extends Command {
    private Kicker kicker;

    public Kick(Kicker kicker) {
        this.kicker = kicker;

        //   addRequirements();
    }

    @Override
    public void execute() {
        kicker.start();
    }

    @Override
    public void end(boolean interrupted) {
        kicker.stop();
    }
}
