package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final Funnel funnel;
    private final Pivot pivot;

    public Intake() {
        System.out.println("[Init] Creating Intake");

        funnel = new Funnel();
        pivot = new Pivot();
    }

    public Funnel getFunnel() {
        return funnel;
    }

    public Pivot getPivot() {
        return pivot;
    }
}
