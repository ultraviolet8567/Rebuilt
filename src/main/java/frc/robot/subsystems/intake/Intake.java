package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Groups the intake's pivot and funnel. Plain holder, not a subsystem -- see {@code Shooter}. */
public class Intake {
    private final Pivot pivot;
    private final Funnel funnel;

    public Intake(Pivot pivot, Funnel funnel) {
        this.pivot = pivot;
        this.funnel = funnel;
    }

    public Pivot getPivot() {
        return pivot;
    }

    public Funnel getFunnel() {
        return funnel;
    }

    /** Deploy and collect: one button, both mechanisms, stowed again on release. */
    public Command collect() {
        return Commands.parallel(pivot.goTo(IntakeConstants.kPivotDeployedRad), funnel.run(false))
                .withName("Collect");
    }
}
