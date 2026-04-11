package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

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

    public double getDriveVoltage() {
        return driveMotor.getBusVoltage();
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderAngle());
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
}
