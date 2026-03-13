package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.Pivot;
import org.littletonrobotics.junction.Logger;

public class LowerPivot extends Command {
    private final Pivot pivot;

    public LowerPivot(Pivot pivot) {
        this.pivot = pivot;
    }

    @Override
    public void execute() {
        pivot.setPivotRads(IntakeConstants.kPivotUpper);
        Logger.recordOutput("Intake/Pivot/Running", true);
/    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
        Logger.recordOutput("Intake/Pivot/Running", false);
    }
}
