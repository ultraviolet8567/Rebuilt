package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

/**
 * Groups the three shooter mechanisms and owns the commands that need more than one of them.
 *
 * <p>This is not a {@code SubsystemBase}. The old {@code Shooter}, {@code Intake} and {@code
 * Storage} classes all extended {@code SubsystemBase} while owning further subsystems, so the
 * command scheduler registered and ran {@code periodic()} on a container that had no hardware and
 * no requirements -- and a command that declared the container as a requirement would not have
 * locked out the mechanisms inside it. Making it a plain holder removes the ambiguity.
 */
public class Shooter {
    private final Flywheel flywheel;
    private final Hood hood;
    private final Kicker kicker;

    public Shooter(Flywheel flywheel, Hood hood, Kicker kicker) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.kicker = kicker;
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

    /** Spin up for a shot from the given distance, adjusting continuously as the robot moves. */
    public Command rangedShot(DoubleSupplier distanceMeters) {
        return Commands.parallel(
                        flywheel.runAtRpm(
                                () -> Flywheel.rpmForDistance(distanceMeters.getAsDouble())),
                        hood.goTo(() -> Hood.angleForDistance(distanceMeters.getAsDouble())))
                .withName("Ranged shot");
    }

    /** Fixed close-range shot at the tunable shuffle velocity. */
    public Command shuffleShot() {
        return Commands.parallel(
                        flywheel.runAtRpm(ShooterConstants.kShuffleRpm::get),
                        hood.goTo(ShooterConstants.kHoodUpperRad))
                .withName("Shuffle shot");
    }
}
