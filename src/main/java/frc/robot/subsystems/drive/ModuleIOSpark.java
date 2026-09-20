package frc.robot.subsystems.drive;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.DoubleSupplier;

/**
 * One MAXSwerve module: Spark Flex on the drive, Spark Max on the steering, and an analog absolute
 * encoder reading the steering angle directly.
 *
 * <p>Two things changed from the previous implementation:
 *
 * <ul>
 *   <li><b>The drive motor is closed loop.</b> It was {@code motor.set(speed / maxSpeed)}, an open
 *       duty cycle that only produces the requested speed on a fresh battery with no load. The
 *       Spark now runs its own velocity loop with a kS/kV feedforward, so the wheel turns at the
 *       speed the kinematics assumed even as the battery sags -- which is also what keeps odometry
 *       honest.
 *   <li><b>Steering feedback is debounced and checked.</b> A failed CAN read used to be
 *       indistinguishable from an encoder reading zero; now it raises a sticky fault, the module
 *       reports itself disconnected, and the driver sees an alert.
 * </ul>
 *
 * <p>Steering stays on a roboRIO {@link PIDController} rather than the Spark's internal loop
 * because the angle comes from an analog encoder wired to the roboRIO, not from a sensor the Spark
 * can see.
 */
public class ModuleIOSpark implements ModuleIO {
    private final SparkFlex driveSpark;
    private final SparkMax turnSpark;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final SparkClosedLoopController driveController;
    private final AnalogInput absoluteEncoder;

    private final PIDController turnPid;
    private final double absoluteOffsetRad;

    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);

    private boolean configurationFailed = false;

    public ModuleIOSpark(int index) {
        absoluteOffsetRad = DriveConstants.kAbsoluteEncoderOffsetsRad[index];

        absoluteEncoder = new AnalogInput(DriveConstants.kAbsoluteEncoderPorts[index]);
        driveSpark = new SparkFlex(DriveConstants.kDriveMotorIds[index], MotorType.kBrushless);
        turnSpark = new SparkMax(DriveConstants.kTurnMotorIds[index], MotorType.kBrushless);
        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getEncoder();
        driveController = driveSpark.getClosedLoopController();

        sparkStickyFault = false;

        var driveConfig = new SparkFlexConfig();
        driveConfig
                .inverted(DriveConstants.kDriveInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DriveConstants.kDriveCurrentLimitAmps)
                .voltageCompensation(12.0);
        driveConfig
                .encoder
                .positionConversionFactor(DriveConstants.kDrivePositionFactor)
                .velocityConversionFactor(DriveConstants.kDriveVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfig.closedLoop.pid(DriveConstants.kDriveKp, 0.0, DriveConstants.kDriveKd);
        driveConfig.closedLoop.feedForward.sv(DriveConstants.kDriveKs, DriveConstants.kDriveKv);
        tryUntilOk(
                driveSpark,
                5,
                () ->
                        driveSpark.configure(
                                driveConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));
        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(DriveConstants.kTurnInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DriveConstants.kTurnCurrentLimitAmps)
                .voltageCompensation(12.0);
        turnConfig
                .encoder
                .positionConversionFactor(DriveConstants.kTurnPositionFactor)
                .velocityConversionFactor(DriveConstants.kTurnVelocityFactor);
        tryUntilOk(
                turnSpark,
                5,
                () ->
                        turnSpark.configure(
                                turnConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));

        configurationFailed = sparkStickyFault;

        turnPid = new PIDController(DriveConstants.kTurnKp, 0.0, DriveConstants.kTurnKd);
        turnPid.enableContinuousInput(-Math.PI, Math.PI);

        tryUntilOk(turnSpark, 5, () -> turnEncoder.setPosition(readAbsoluteAngleRad()));
    }

    /** True if any Spark refused its configuration at boot. */
    public boolean configurationFailed() {
        return configurationFailed;
    }

    private double readAbsoluteAngleRad() {
        double fraction = absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V();
        double angle = fraction * 2 * Math.PI + absoluteOffsetRad;
        angle = MathUtil.inputModulus(angle, -Math.PI, Math.PI);
        return angle * (DriveConstants.kAbsoluteEncoderInverted ? -1 : 1);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(driveSpark, driveEncoder::getPosition, v -> inputs.drivePositionMeters = v);
        ifOk(driveSpark, driveEncoder::getVelocity, v -> inputs.driveVelocityMetersPerSec = v);
        ifOk(
                driveSpark,
                new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
                v -> inputs.driveAppliedVolts = v[0] * v[1]);
        ifOk(driveSpark, driveSpark::getOutputCurrent, v -> inputs.driveCurrentAmps = v);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        ifOk(turnSpark, turnEncoder::getVelocity, v -> inputs.turnVelocityRadPerSec = v);
        ifOk(
                turnSpark,
                new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
                v -> inputs.turnAppliedVolts = v[0] * v[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, v -> inputs.turnCurrentAmps = v);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        double volts = absoluteEncoder.getAverageVoltage();
        inputs.absoluteEncoderVolts = volts;
        // A disconnected analog encoder floats at (or very near) 0 V. The real encoder never
        // reaches the rails, so this distinguishes "unplugged" from "pointing at zero".
        inputs.absoluteEncoderConnected = encoderConnectedDebounce.calculate(volts > 0.05);
        inputs.turnPosition = new Rotation2d(readAbsoluteAngleRad());
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSec) {
        driveController.setSetpoint(
                velocityMetersPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSpark.setVoltage(volts);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnSpark.setVoltage(
                MathUtil.clamp(
                        turnPid.calculate(readAbsoluteAngleRad(), rotation.getRadians()),
                        -12.0,
                        12.0));
    }

    @Override
    public void stop() {
        driveSpark.stopMotor();
        turnSpark.stopMotor();
    }

    @Override
    public void setBrakeMode(boolean brake) {
        var config = new SparkFlexConfig();
        config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        tryUntilOk(
                driveSpark,
                5,
                () ->
                        driveSpark.configure(
                                config,
                                ResetMode.kNoResetSafeParameters,
                                PersistMode.kNoPersistParameters));
    }
}
