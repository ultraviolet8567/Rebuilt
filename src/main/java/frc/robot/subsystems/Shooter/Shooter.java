package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final Flywheel flywheel;
    private final Hood hood;
    private final Kicker kicker;

    public Shooter() {
        flywheel = new Flywheel();
        hood = new Hood();
        kicker = new Kicker();
    }

    public Flywheel getFlywheel() {
        return flywheel;
    }

    public Hood getHood() {
        return hood;
    }

    public Kicker getKicker() {
        return kicker;
    }
}
