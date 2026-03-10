package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final Mouth mouth;

    public Intake() {
        mouth = new Mouth();
    }

    public Mouth getMouth() {
        return mouth;
    }
}
