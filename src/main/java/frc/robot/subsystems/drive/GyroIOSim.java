package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/**
 * Integrates the chassis rotation rate the simulated modules are producing.
 *
 * <p>The previous version pushed this through the Pigeon's own {@code Pigeon2SimState}, which meant
 * the simulation depended on a Phoenix background thread publishing before the next read -- the
 * smoke test had to sprinkle {@code Timer.delay(0.002)} between loops to give it time. A plain
 * integrator has no such race, so the simulation is deterministic and the tests run at full speed.
 */
public class GyroIOSim implements GyroIO {
    private final DoubleSupplier yawRateRadPerSec;
    private double yawRad = 0.0;

    public GyroIOSim(DoubleSupplier yawRateRadPerSec) {
        this.yawRateRadPerSec = yawRateRadPerSec;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        double rate = yawRateRadPerSec.getAsDouble();
        yawRad += rate * Constants.kLoopPeriodSecs;

        inputs.connected = true;
        inputs.yawPosition = new Rotation2d(yawRad);
        inputs.yawVelocityRadPerSec = rate;
    }

    @Override
    public void setYaw(Rotation2d value) {
        yawRad = value.getRadians();
    }
}
