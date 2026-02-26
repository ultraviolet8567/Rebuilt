package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public final Flywheel flywheel;
    public final Hood hood;
    public final Kicker kicker;

    public Shooter() {
        flywheel = new Flywheel();
        hood = new Hood();
        kicker = new Kicker();
    }
}
