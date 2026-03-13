package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Hood;

public class RunHood extends Command {
    private Hood hood;

    public RunHood(Hood hood) {
        this.hood = hood;
    }

    @Override
    public void execute() {
        hood.setHoodRads(hood.getTargetPosition());
    }

    @Override
    public void end(boolean interrupted) {
        hood.stop();
    }
}
