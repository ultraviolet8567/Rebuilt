package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Pivot;

public class SetPivot extends Command {
    private final Pivot pivot;
    private final double angle;

    public SetPivot(Pivot pivot, double angle) {
        this.pivot = pivot;
        this.angle = angle;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.setTargetPosition(angle);
    }
}
