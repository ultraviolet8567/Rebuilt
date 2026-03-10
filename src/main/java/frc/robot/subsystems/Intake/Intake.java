package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final Raiser raiser;
    private final Mouth mouth;

    public Intake() {
        raiser = new Raiser();
        mouth = new Mouth();
    }

    public Raiser getRaiser() {
        return raiser;
    }

    public Mouth getMouth() {
        return mouth;
    }
}
