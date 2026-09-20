package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.util.SimBattery;

// Encoder: Thing that is above wheel and records how much it moves.
public class SwerveModule {
    private final SparkFlex driveMotor;
    private final SparkMax turningMotor;
    private final SparkFlexConfig driveConfig;
    private final SparkMaxConfig turningConfig;
    private final PIDController turningPidController;
    public SwerveModulePosition modulePosition;
    private final AnalogInput absoluteEncoder;
    private double ConfigOffset;
    private final boolean ConfigReversed;

    // ---- Desktop simulation only (null on the real robot) ----
    // Physics models of the two motors + their gearboxes.
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;
    // REV's simulated Spark controllers. They read the duty cycle / voltage the robot code
    // commanded and let us push a simulated encoder velocity/position back in.
    private final SparkFlexSim driveSparkSim;
    private final SparkMaxSim turnSparkSim;
    // Lets us set the voltage the analog absolute encoder "sees".
    private final AnalogInputSim absoluteEncoderSim;

    public SwerveModule(
            int driveMotorID,
            int turningMotorID,
            boolean driveMotorReversed,
            boolean turningMotorReversed,
            int absoluteEncoderID,
            double ConfigOffset,
            boolean ConfigReversed,
            double driveGearRatio,
            double driveRot2Meter,
            double driveRPM2MeterPerSec) {
        this.ConfigOffset = ConfigOffset;
        this.ConfigReversed = ConfigReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderID);
        driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

        driveConfig = new SparkFlexConfig();
        turningConfig = new SparkMaxConfig();

        driveConfig.voltageCompensation(12.0);
        driveConfig.smartCurrentLimit(40);
        driveConfig.inverted(driveMotorReversed);
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.encoder.positionConversionFactor(driveRot2Meter);
        driveConfig.encoder.velocityConversionFactor(driveRPM2MeterPerSec);

        turningConfig.voltageCompensation(12.0);
        turningConfig.smartCurrentLimit(40);
        turningConfig.idleMode(IdleMode.kBrake);
        turningConfig.inverted(turningMotorReversed);
        turningConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningConfig.encoder.velocityConversionFactor(
                ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(
                turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isSimulation()) {
            DCMotor driveGearbox = DCMotor.getNeoVortex(1);
            DCMotor turnGearbox = DCMotor.getNEO(1);
            // driveGearRatio is stored as wheel-turns-per-motor-turn (e.g. 1/6.03); the sim
            // wants motor-turns-per-wheel-turn.
            driveSim =
                    new DCMotorSim(
                            LinearSystemId.createDCMotorSystem(
                                    driveGearbox,
                                    SimConstants.kDriveWheelMOI,
                                    1.0 / driveGearRatio),
                            driveGearbox);
            turnSim =
                    new DCMotorSim(
                            LinearSystemId.createDCMotorSystem(
                                    turnGearbox,
                                    SimConstants.kTurnMOI,
                                    1.0 / ModuleConstants.kTurningMotorGearRatio),
                            turnGearbox);
            driveSparkSim = new SparkFlexSim(driveMotor, driveGearbox);
            turnSparkSim = new SparkMaxSim(turningMotor, turnGearbox);
            absoluteEncoderSim = new AnalogInputSim(absoluteEncoder);

            // Start with the wheel pointing straight ahead.
            setSimAbsoluteEncoderAngle(0.0);
        } else {
            driveSim = null;
            turnSim = null;
            driveSparkSim = null;
            turnSparkSim = null;
            absoluteEncoderSim = null;
        }

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition();
    }

    public double getTurningPosition() {
        return getAbsoluteEncoderAngle();
        // return turningMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity();
    }

    public double getAbsoluteEncoderAngle() {
        double angle = absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V();
        angle *= 2 * Math.PI;
        angle += ConfigOffset;
        angle = MathUtil.inputModulus(angle, -Math.PI, Math.PI);

        return angle * (ConfigReversed ? -1 : 1);
    }

    public double getAbsoluteEncoderVoltage() {
        return absoluteEncoder.getAverageVoltage();
    }

    /** Voltage actually applied to the drive motor (was returning the bus voltage before). */
    public double getDriveVoltage() {
        return driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderAngle());
        // REV's simulated Sparks keep their own copy of the position; keep them in step.
        if (driveSparkSim != null) {
            driveSparkSim.setPosition(0);
            turnSparkSim.setPosition(getAbsoluteEncoderAngle());
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModuleState getAbsoluteState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderAngle()));
    }

    public void setDesiredState(SwerveModuleState state, double throttle) {
        state.optimize(getState().angle);
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
        } else {
            // Logger.recordOutput("SwerveModule/" + turningMotor.getDeviceId() + "/State", state);
            driveMotor.set(
                    state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            turningMotor.set(
                    turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        }
    }

    public void setModuleRotation(SwerveModuleState state) {
        driveMotor.set(0);
        turningMotor.set(
                turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getState().angle);
    }

    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }

    // ------------------------------------------------------------------------------------
    // Desktop simulation
    // ------------------------------------------------------------------------------------

    /**
     * Advance this module's physics by one loop. Called from Swerve.simulationPeriodic(). Does
     * nothing on the real robot.
     */
    public void simulationPeriodic() {
        if (driveSim == null) return;

        double dt = Constants.kLoopPeriodSecs;
        double vbus = RoboRioSim.getVInVoltage();

        // Drive motor: robot code commanded a duty cycle -> voltage -> wheel spins.
        driveSim.setInputVoltage(driveSparkSim.getAppliedOutput() * vbus);
        driveSim.update(dt);
        double wheelMetersPerSec =
                driveSim.getAngularVelocityRadPerSec() * (ModuleConstants.kWheelDiameterMeters / 2);
        // The drive encoder is configured to report m/s and meters, so hand it m/s.
        driveSparkSim.iterate(wheelMetersPerSec, vbus, dt);

        // Turning motor: same idea, then reflect the new angle in the absolute encoder.
        turnSim.setInputVoltage(turnSparkSim.getAppliedOutput() * vbus);
        turnSim.update(dt);
        // The turning encoder is configured to report rad/s and rad.
        turnSparkSim.iterate(turnSim.getAngularVelocityRadPerSec(), vbus, dt);
        setSimAbsoluteEncoderAngle(turnSim.getAngularPositionRad());

        SimBattery.addCurrent(driveSim.getCurrentDrawAmps());
        SimBattery.addCurrent(turnSim.getCurrentDrawAmps());
    }

    /**
     * Set the analog absolute encoder so that getAbsoluteEncoderAngle() returns the given angle.
     * This is the inverse of the math in getAbsoluteEncoderAngle().
     */
    private void setSimAbsoluteEncoderAngle(double angleRad) {
        double raw = angleRad * (ConfigReversed ? -1 : 1) - ConfigOffset;
        double fraction = MathUtil.inputModulus(raw / (2 * Math.PI), 0.0, 1.0);
        absoluteEncoderSim.setVoltage(fraction * RobotController.getVoltage5V());
    }
}
