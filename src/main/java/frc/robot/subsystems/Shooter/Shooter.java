package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final Flywheel flywheel;
    private final Hood hood;
    private final Kicker kicker;

    public Shooter() {
        System.out.println("[Init] Creating Shooter");

        flywheel = new Flywheel();
        hood = new Hood();
        kicker = new Kicker();
    }

    @Override
    public void periodic() {
        flywheel.periodic();
        hood.periodic();
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
