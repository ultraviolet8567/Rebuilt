package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Hood;

public class SetHood extends Command {
    private final Hood hood;
    private final boolean reversed;

    public SetHood(Hood hood, boolean reversed) {
        this.hood = hood;
        this.reversed = reversed;
    }

    @Override
    public void execute() {
        hood.setTargetPosition(hood.getTargetPosition() + (reversed ? -0.01 : 0.01));
    }
}
