package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.Pivot;

public class Feed extends Command {
    private final Pivot pivot;
    private int count;

    public Feed(Pivot pivot) {
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
        pivot.setTargetPosition(IntakeConstants.kPivotLower);
        count = 0;
    }

    @Override
    public void execute() {
        count++;
        if (count % 50 > 25) {
            pivot.setTargetPosition(IntakeConstants.kPivotLower);
        } else {
            pivot.setTargetPosition(IntakeConstants.kPivotUpper);
        }
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setTargetPosition(IntakeConstants.kPivotLower);
    }
}
