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
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Odometry;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
    private final SparkFlex leadMotor, followerMotor;
    private final SparkFlexConfig globalConfig, followerConfig;
    private final RelativeEncoder leadEncoder, followerEncoder;
    private final PIDController leadPIDController, followerPIDController;
    private final SimpleMotorFeedforward leadFeedforwardController, followerFeedforwardController;


    private double targetVelocity;
    private boolean running;

    public Flywheel() {
        System.out.println("[Init] Creating Flywheel");

        leadMotor = new SparkFlex(CAN.kFlywheelLeadPort, MotorType.kBrushless);
        leadEncoder = leadMotor.getEncoder();
        followerMotor = new SparkFlex(CAN.kFlywheelFollowerPort, MotorType.kBrushless);
        followerEncoder = followerMotor.getEncoder();

        globalConfig = new SparkFlexConfig();
        globalConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.kFlywheelReduction);
        globalConfig.smartCurrentLimit(80);
        globalConfig.idleMode(IdleMode.kCoast);

        followerConfig = new SparkFlexConfig();
        followerConfig.apply(globalConfig); // .follow(leadMotor);

        leadMotor.configure(
                globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(
                globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leadPIDController =
                new PIDController(
                        ShooterConstants.kFlywheelP.get(),
                        ShooterConstants.kFlywheelI.get(),
                        ShooterConstants.kFlywheelD.get());
        leadFeedforwardController =
                new SimpleMotorFeedforward(
                        ShooterConstants.kFlywheelS.get(),
                        ShooterConstants.kFlywheelV.get(),
                        ShooterConstants.kFlywheelA.get());

        followerPIDController =
                new PIDController(
                        ShooterConstants.kFlywheelP.get(),
                        ShooterConstants.kFlywheelI.get(),
                        ShooterConstants.kFlywheelD.get());
        followerFeedforwardController =
                new SimpleMotorFeedforward(
                        ShooterConstants.kFlywheelS.get(),
                        ShooterConstants.kFlywheelV.get(),
                        ShooterConstants.kFlywheelA.get());

        targetVelocity = ShooterConstants.kFlywheelMaxVelocity;
        running = false;
    }

    public void periodic() {
        leadPIDController.setP(ShooterConstants.kFlywheelP.get());
        leadPIDController.setI(ShooterConstants.kFlywheelI.get());
        leadPIDController.setD(ShooterConstants.kFlywheelD.get());

        followerPIDController.setP(ShooterConstants.kFlywheelP.get());
        followerPIDController.setI(ShooterConstants.kFlywheelI.get());
        followerPIDController.setD(ShooterConstants.kFlywheelD.get());

        leadFeedforwardController.setKs(ShooterConstants.kFlywheelS.get());
        leadFeedforwardController.setKv(ShooterConstants.kFlywheelV.get());
        leadFeedforwardController.setKa(ShooterConstants.kFlywheelA.get());

        followerFeedforwardController.setKs(ShooterConstants.kFlywheelS.get());
        followerFeedforwardController.setKv(ShooterConstants.kFlywheelV.get());
        followerFeedforwardController.setKa(ShooterConstants.kFlywheelA.get());

        Logger.recordOutput("Shooter/Flywheel/Lead/Velocity", getVelocity(leadMotor));
        Logger.recordOutput("Shooter/Flywheel/Follower/Velocity", getVelocity(followerMotor));
        Logger.recordOutput(
                "Shooter/Flywheel/Lead/MeasuredVoltage",
                leadMotor.getAppliedOutput() * leadMotor.getBusVoltage());
        Logger.recordOutput(
                "Shooter/Flywheel/Follower/MeasuredVoltage",
                followerMotor.getAppliedOutput() * followerMotor.getBusVoltage());

        Logger.recordOutput("Shooter/Flywheel/Ks", leadFeedforwardController.getKs());
        Logger.recordOutput("Shooter/Flywheel/Kv", leadFeedforwardController.getKv());
        Logger.recordOutput("Shooter/Flywheel/Ka", leadFeedforwardController.getKa());

        if (running) {
            setFlywheelRadsPerSec(targetVelocity);
        } else {
            stop();
        }
    }

    public double getVelocity(SparkFlex motor) {
        return motor.getEncoder().getVelocity();
    }

    public void setFlywheelRadsPerSec(double targetVelocity) {
        double pidVoltage, ffVoltage;

        pidVoltage = leadPIDController.calculate(-getVelocity(leadMotor), targetVelocity);
        ffVoltage = leadFeedforwardController.calculate(targetVelocity);

        Logger.recordOutput("Shooter/Flywheel/Lead/pidVoltage", pidVoltage);
        Logger.recordOutput("Shooter/Flywheel/Lead/ffVoltage", ffVoltage);

        setMotorVoltage(leadMotor, pidVoltage + ffVoltage);

        pidVoltage = followerPIDController.calculate(-getVelocity(followerMotor), targetVelocity);
        ffVoltage = followerFeedforwardController.calculate(targetVelocity);

        Logger.recordOutput("Shooter/Flywheel/Follower/pidVoltage", pidVoltage);
        Logger.recordOutput("Shooter/Flywheel/Follower/ffVoltage", ffVoltage);

        setMotorVoltage(followerMotor, pidVoltage + ffVoltage);
    }

    public void setMotorVoltage(SparkFlex motor, double voltage) {
        voltage =
                MathUtil.clamp(
                        voltage,
                        -ShooterConstants.kFlywheelVoltage,
                        ShooterConstants.kFlywheelVoltage);
        voltage *= ShooterConstants.kFlywheelInverted ? -1 : 1;

        motor.setVoltage(voltage);
    }

    public void setFlywheelVoltage(double voltage) {
        setMotorVoltage(leadMotor, voltage);
        setMotorVoltage(followerMotor, voltage);
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
        // return Math.abs(getVelocity(leadMotor)) > Math.abs(velocity)
        //        && Math.abs(getVelocity(followerMotor)) > Math.abs(velocity);
        return Math.abs(Math.abs(velocity) - Math.abs(getVelocity(leadMotor)))
                < ShooterConstants.kFlywheelVelocityTolerance;
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public void setTargetVelocity() {
        targetVelocity = calculateTargetVelocity(Odometry.getDistToRobot());
    }

    public double calculateTargetVelocity(double dist) {
        return Math.sqrt((FieldConstants.kG * dist * dist)
            / (2 * Math.pow(Math.cos(ShooterConstants.kShooterAngle), 2) 
            * (Math.tan(ShooterConstants.kShooterAngle) * dist - FieldConstants.kHubHeightDiff)));
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public SparkFlex getLeadMotor() {
        return leadMotor;
    }

    public SparkFlex getFollowerMotor() {
        return followerMotor;
    }
}
