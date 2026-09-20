package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.Logger;

/**
 * The small set of facts several unrelated parts of the robot need to agree on: what mode we are
 * in, whether the shooter is ready, whether demo mode is on.
 *
 * <p>Previously these lived as public mutable fields on the {@code Lights} singleton, so {@code
 * Swerve.getMaxSpeed()} and {@code Flywheel.getTargetVelocity()} both had to reach into the LED
 * subsystem to find out whether demo mode was enabled -- the drivetrain's top speed depended on the
 * light strip. Naming the shared state for what it is removes that coupling and leaves Lights as a
 * pure consumer. 11 of the 42 corpus robots keep an explicit RobotState for exactly this (FRC 341,
 * 6002, 3467 among them).
 */
public final class RobotState {
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private RobotState() {}

    public static enum Mode {
        DISABLED,
        AUTO,
        TELEOP,
        TEST
    }

    private Mode mode = Mode.DISABLED;
    private Alliance alliance = Alliance.Blue;
    private boolean demoMode = false;
    private boolean shooterAtSpeed = false;
    private boolean intakeDeployed = false;
    private boolean wheelsLocked = false;
    private boolean lowBattery = false;

    /** Refresh the facts we can read ourselves, and log the rest. Called once per loop. */
    public void update() {
        DriverStation.getAlliance().ifPresent(a -> alliance = a);

        Logger.recordOutput("RobotState/Mode", mode);
        Logger.recordOutput("RobotState/Alliance", alliance);
        Logger.recordOutput("RobotState/DemoMode", demoMode);
        Logger.recordOutput("RobotState/ShooterAtSpeed", shooterAtSpeed);
        Logger.recordOutput("RobotState/IntakeDeployed", intakeDeployed);
        Logger.recordOutput("RobotState/WheelsLocked", wheelsLocked);
        Logger.recordOutput("RobotState/LowBattery", lowBattery);
    }

    public Mode getMode() {
        return mode;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    /** True when the driver station reports we are on the red alliance. */
    public boolean isRedAlliance() {
        return alliance == Alliance.Red;
    }

    /** Demo mode caps drivetrain speed and shooter velocity for crowd demonstrations. */
    public boolean isDemoMode() {
        return demoMode;
    }

    public void setDemoMode(boolean demoMode) {
        this.demoMode = demoMode;
    }

    public boolean isShooterAtSpeed() {
        return shooterAtSpeed;
    }

    public void setShooterAtSpeed(boolean shooterAtSpeed) {
        this.shooterAtSpeed = shooterAtSpeed;
    }

    public boolean isIntakeDeployed() {
        return intakeDeployed;
    }

    public void setIntakeDeployed(boolean intakeDeployed) {
        this.intakeDeployed = intakeDeployed;
    }

    public boolean areWheelsLocked() {
        return wheelsLocked;
    }

    public void setWheelsLocked(boolean wheelsLocked) {
        this.wheelsLocked = wheelsLocked;
    }

    public boolean isLowBattery() {
        return lowBattery;
    }

    public void setLowBattery(boolean lowBattery) {
        this.lowBattery = lowBattery;
    }
}
