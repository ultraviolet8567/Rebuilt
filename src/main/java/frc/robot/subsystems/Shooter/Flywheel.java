package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
    private final SparkFlex leadMotor, followerMotor;
    private final SparkFlexConfig globalConfig, followerConfig;
    private final RelativeEncoder relativeEncoder;
    private final PIDController pidController;
    private final SimpleMotorFeedforward feedforwardController;

    private double targetVelocity;
    private boolean running;

    public Flywheel() {
        System.out.println("[Init] Creating Flywheel");

        leadMotor = new SparkFlex(CAN.kFlywheelLeadPort, MotorType.kBrushless);
        relativeEncoder = leadMotor.getEncoder();
        followerMotor = new SparkFlex(CAN.kFlywheelFollowerPort, MotorType.kBrushless);

        globalConfig = new SparkFlexConfig();
        globalConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kFlywheelReduction);
        globalConfig.smartCurrentLimit(80);
        globalConfig.idleMode(IdleMode.kCoast);

        followerConfig = new SparkFlexConfig();
        followerConfig.apply(globalConfig); // .follow(leadMotor);

        leadMotor.configure(
                globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController =
                new PIDController(
                        ShooterConstants.kFlywheelP.get(),
                        ShooterConstants.kFlywheelI.get(),
                        ShooterConstants.kFlywheelD.get());
        feedforwardController =
                new SimpleMotorFeedforward(
                        ShooterConstants.kFlywheelS.get(),
                        ShooterConstants.kFlywheelV.get(),
                        ShooterConstants.kFlywheelA.get());

        targetVelocity = ShooterConstants.kFlywheelMaxVelocity;
        running = false;

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> pidController.setP(ShooterConstants.kFlywheelP.get()),
                ShooterConstants.kFlywheelP);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> pidController.setP(ShooterConstants.kFlywheelI.get()),
                ShooterConstants.kFlywheelI);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> pidController.setP(ShooterConstants.kFlywheelD.get()),
                ShooterConstants.kFlywheelD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> feedforwardController.setKs(ShooterConstants.kFlywheelS.get()),
                ShooterConstants.kFlywheelS);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> feedforwardController.setKv(ShooterConstants.kFlywheelV.get()),
                ShooterConstants.kFlywheelV);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> feedforwardController.setKa(ShooterConstants.kFlywheelA.get()),
                ShooterConstants.kFlywheelA);
    }

    public void periodic() {
        Logger.recordOutput("Shooter/Flywheel/Velocity", getVelocity());
        Logger.recordOutput(
                "Shooter/Flywheel/MeasuredVoltage",
                leadMotor.getAppliedOutput() * leadMotor.getBusVoltage());

        Logger.recordOutput("Shooter/Flywheel/Ks", feedforwardController.getKs());
        Logger.recordOutput("Shooter/Flywheel/Kv", feedforwardController.getKv());

        if (running) {
            setFlywheelRadsPerSec(targetVelocity);
        } else {
            stop();
        }
    }

    public double getVelocity() {
        return relativeEncoder.getVelocity();
    }

    public void setFlywheelRadsPerSec(double targetVelocity) {
        double pidVoltage = pidController.calculate(getVelocity(), targetVelocity);
        double ffVoltage = feedforwardController.calculate(targetVelocity);

        Logger.recordOutput("Shooter/Flywheel/pidVoltage", pidVoltage);
        Logger.recordOutput("Shooter/Flywheel/ffVoltage", ffVoltage);
        setFlywheelVoltage(pidVoltage + ffVoltage);
    }

    public void setFlywheelVoltage(double voltage) {
        voltage =
                MathUtil.clamp(
                        voltage,
                        -ShooterConstants.kFlywheelVoltage,
                        ShooterConstants.kFlywheelVoltage);
        voltage *= ShooterConstants.kFlywheelInverted ? -1 : 1;

        leadMotor.setVoltage(voltage);
    }

    public void stop() {
        running = false;
        setFlywheelVoltage(0);
    }

    public void start(double velocity) {
        running = true;
        setTargetVelocity(velocity);
    }

    public boolean atVelocity(double velocity) {
        return Math.abs(getVelocity()) > Math.abs(velocity);
        // return Math.abs(Math.abs(velocity) - Math.abs(getVelocity()))
        //        < ShooterConstants.kFlywheelVelocityTolerance;
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }
}
