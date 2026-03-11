package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.Pivot;

public class StartPivot extends Command {
    private final Pivot pivot;

    public StartPivot(Pivot pivot) {
        this.pivot = pivot;
    }

    @Override
    public void execute() {
        pivot.setPivotRads(IntakeConstants.kPivotExtendedAngle);
    }
}
