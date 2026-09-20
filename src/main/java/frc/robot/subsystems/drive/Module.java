package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.Alerts;
import org.littletonrobotics.junction.Logger;

/** Hardware-independent behaviour of one swerve module. */
public class Module {
    private static final String[] kNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final String name;

    private final Alert driveDisconnected;
    private final Alert turnDisconnected;
    private final Alert encoderDisconnected;

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.name = kNames[index];
        driveDisconnected = Alerts.create("Drive motor disconnected: " + name, AlertType.kError);
        turnDisconnected = Alerts.create("Turn motor disconnected: " + name, AlertType.kError);
        encoderDisconnected =
                Alerts.create("Steering encoder disconnected: " + name, AlertType.kError);
    }

    /** Read sensors and publish them. Called from {@link Drive#periodic()}. */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/" + name, inputs);

        driveDisconnected.set(!inputs.driveConnected);
        turnDisconnected.set(!inputs.turnConnected);
        encoderDisconnected.set(!inputs.absoluteEncoderConnected);
    }

    /**
     * Drive this module to the requested state.
     *
     * <p>Two corrections are applied that the previous code did not make:
     *
     * <ul>
     *   <li>{@code optimize} -- already present: never steer more than 90 degrees, reverse the
     *       wheel instead.
     *   <li><b>{@code cosineScale}</b> -- new. While a wheel is still rotating towards its target
     *       angle, any speed it is commanded pushes the robot sideways. Scaling the commanded speed
     *       by the cosine of the remaining steering error makes the wheel wait until it is pointing
     *       roughly the right way before it pushes hard. The visible effect is that the robot stops
     *       lurching off-heading at the start of a hard direction change. 26 of the 42 corpus
     *       robots do this.
     * </ul>
     */
    public void runSetpoint(SwerveModuleState state) {
        state.optimize(getAngle());
        state.cosineScale(getAngle());

        io.setTurnPosition(state.angle);
        io.setDriveVelocity(state.speedMetersPerSecond);

        Logger.recordOutput("Drive/" + name + "/Setpoint", state);
    }

    /** Point the wheel at an angle without driving it. Used by the X-lock. */
    public void runTurnOnly(Rotation2d angle) {
        io.setTurnPosition(angle);
        io.setDriveVelocity(0.0);
    }

    /** Open-loop drive voltage with the wheel held straight. Used for characterisation. */
    public void runCharacterization(double volts) {
        io.setDriveVoltage(volts);
        io.setTurnPosition(Rotation2d.kZero);
    }

    public void stop() {
        io.stop();
    }

    public void setBrakeMode(boolean brake) {
        io.setBrakeMode(brake);
    }

    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    public double getPositionMeters() {
        return inputs.drivePositionMeters;
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityMetersPerSec;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Wheel rotation in radians, for wheel-radius characterisation. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionMeters / DriveConstants.kWheelRadiusMeters;
    }

    public boolean isConnected() {
        return inputs.driveConnected && inputs.turnConnected && inputs.absoluteEncoderConnected;
    }
}
