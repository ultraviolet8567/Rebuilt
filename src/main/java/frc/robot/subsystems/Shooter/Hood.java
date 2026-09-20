package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.util.SimBattery;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Hood extends SubsystemBase {
    private final SparkMax hoodMotor;
    private final SparkMaxConfig hoodMotorConfig;
    private final RelativeEncoder hoodEncoder;
    private final DutyCycleEncoder absoluteEncoder;
    private final PIDController pidController;
    // private final SimpleMotorFeedforward feedforwardController;

    private double targetPosition;

    // Total reduction from motor shaft to hood angle.
    private static final double kTotalReduction =
            ShooterConstants.kHoodGearboxReduction * ShooterConstants.kHoodRackReduction;

    // Mechanism2d drawing for AdvantageScope (real robot and sim).
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
    private final LoggedMechanismLigament2d hoodLigament;

    // Desktop simulation only (null on the real robot).
    private final DCMotorSim hoodSim;
    private final SparkMaxSim hoodSparkSim;
    private final DutyCycleEncoderSim absoluteEncoderSim;

    public Hood() {
        System.out.println("[Init] Creating Hood");

        hoodMotor = new SparkMax(CAN.kHoodPort, MotorType.kBrushless);
        hoodEncoder = hoodMotor.getEncoder();
        hoodMotorConfig = new SparkMaxConfig();
        hoodMotorConfig.encoder.positionConversionFactor(1.0 / kTotalReduction);
        hoodMotorConfig.smartCurrentLimit(50);
        hoodMotorConfig.idleMode(IdleMode.kBrake);
        hoodMotor.configure(
                hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        absoluteEncoder = new DutyCycleEncoder(CAN.kHoodEncoderPort);
        pidController =
                new PIDController(
                        ShooterConstants.kHoodP.get(),
                        ShooterConstants.kHoodI.get(),
                        ShooterConstants.kHoodD.get());

        LoggedMechanismRoot2d root = mechanism.getRoot("Shooter", 0.5, 0.2);
        root.append(
                new LoggedMechanismLigament2d("Barrel", 0.3, 70, 8, new Color8Bit(Color.kGray)));
        hoodLigament =
                root.append(
                        new LoggedMechanismLigament2d(
                                "Hood", 0.25, 90, 6, new Color8Bit(Color.kOrange)));

        if (RobotBase.isSimulation()) {
            DCMotor gearbox = DCMotor.getNEO(1);
            hoodSim =
                    new DCMotorSim(
                            LinearSystemId.createDCMotorSystem(
                                    gearbox, SimConstants.kHoodMOI, kTotalReduction),
                            gearbox);
            hoodSparkSim = new SparkMaxSim(hoodMotor, gearbox);
            absoluteEncoderSim = new DutyCycleEncoderSim(absoluteEncoder);
            absoluteEncoderSim.setConnected(true);

            // Boot with the hood at its lowest position, like a robot that was powered off
            // with the hood retracted. The physics model's angle is the raw motor-side angle
            // (see simulationPeriodic for the sign convention).
            hoodSim.setState(hoodAngleToSimAngle(ShooterConstants.kHoodLower), 0.0);
            setSimAbsoluteEncoderAngle(ShooterConstants.kHoodLower);
        } else {
            hoodSim = null;
            hoodSparkSim = null;
            absoluteEncoderSim = null;
        }

        targetPosition = getAbsoluteRotationRads();

        resetRelativeEncoder();
    }

    public void periodic() {
        Logger.recordOutput("Shooter/Hood/AbsoluteRotation", getAbsoluteRotationRads());
        Logger.recordOutput("Shooter/Hood/RelativeRotation", getRelativeRotationRads());
        Logger.recordOutput("Shooter/Hood/TargetRotation", targetPosition);
        Logger.recordOutput(
                "Shooter/Hood/AppliedVoltage",
                hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage());

        pidController.setP(ShooterConstants.kHoodP.get());
        pidController.setI(ShooterConstants.kHoodI.get());
        pidController.setD(ShooterConstants.kHoodD.get());

        double diff = Math.abs(getAbsoluteRotationRads() - getRelativeRotationRads());
        if (diff > 0.001 && diff < 0.1) resetRelativeEncoder();

        setAngleRads(targetPosition);

        hoodLigament.setAngle(90 + Units.radiansToDegrees(getRelativeRotationRads()));
        Logger.recordOutput("Mechanism2d/Hood", mechanism);
    }

    public double getAbsoluteRotationRads() {
        double angle = absoluteEncoder.get();
        angle *= 2 * Math.PI;
        angle *= ShooterConstants.kHoodAbsoluteEncoderInverted ? -1 : 1;
        angle /= ShooterConstants.kHoodRackReduction;
        angle += ShooterConstants.kHoodEncoderOffset;
        angle = MathUtil.inputModulus(angle, -Math.PI, Math.PI);
        return angle;
    }

    public double getRelativeRotationRads() {
        double angle = hoodEncoder.getPosition();
        angle *= 2 * Math.PI * (ShooterConstants.kHoodRelativeEncoderInverted ? -1 : 1);
        return angle;
    }

    public void resetRelativeEncoder() {
        double angle = getAbsoluteRotationRads();
        angle /= 2 * Math.PI;
        angle *= (ShooterConstants.kHoodRelativeEncoderInverted ? -1 : 1);
        hoodEncoder.setPosition(angle);
        // REV's simulated Spark keeps its own copy of the position and overwrites the encoder
        // with it every loop, so in simulation the sim object has to be told as well.
        if (hoodSparkSim != null) hoodSparkSim.setPosition(angle);
    }

    public void setAngleRads(double angle) {
        double voltage =
                pidController.calculate(
                        getRelativeRotationRads(),
                        MathUtil.clamp(
                                angle, ShooterConstants.kHoodLower, ShooterConstants.kHoodUpper));
        setHoodVoltage(voltage);
    }

    public void setHoodVoltage(double voltage) {
        voltage =
                MathUtil.clamp(
                        voltage, -ShooterConstants.kHoodVoltage, ShooterConstants.kHoodVoltage);
        voltage *= ShooterConstants.kHoodInverted ? -1 : 1;
        hoodMotor.setVoltage(voltage);
    }

    public void stop() {
        setHoodVoltage(0);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition =
                MathUtil.clamp(
                        targetPosition, ShooterConstants.kHoodLower, ShooterConstants.kHoodUpper);
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    // ------------------------------------------------------------------------------------
    // Desktop simulation
    // ------------------------------------------------------------------------------------

    /**
     * The physics model tracks the gearbox output angle as the relative encoder would count it
     * (positive applied voltage = positive count). getRelativeRotationRads() applies the
     * kHoodRelativeEncoderInverted sign on top of that, so hood angle = sign * sim angle.
     */
    private static double hoodAngleToSimAngle(double hoodAngleRad) {
        return hoodAngleRad * (ShooterConstants.kHoodRelativeEncoderInverted ? -1 : 1);
    }

    private static double simAngleToHoodAngle(double simAngleRad) {
        return simAngleRad * (ShooterConstants.kHoodRelativeEncoderInverted ? -1 : 1);
    }

    @Override
    public void simulationPeriodic() {
        double dt = Constants.kLoopPeriodSecs;
        double vbus = RoboRioSim.getVInVoltage();

        hoodSim.setInputVoltage(hoodSparkSim.getAppliedOutput() * vbus);
        hoodSim.update(dt);

        // The relative encoder's velocity conversion factor is left at the default (motor RPM),
        // and its position factor is 1/kTotalReduction, so feed it motor RPM and REV's sim will
        // integrate position in output rotations for us.
        double motorRpm = hoodSim.getAngularVelocityRPM() * kTotalReduction;
        hoodSparkSim.iterate(motorRpm, vbus, dt);

        setSimAbsoluteEncoderAngle(simAngleToHoodAngle(hoodSim.getAngularPositionRad()));

        SimBattery.addCurrent(hoodSim.getCurrentDrawAmps());
    }

    /**
     * Set the duty-cycle absolute encoder so getAbsoluteRotationRads() returns the given hood
     * angle. Because the encoder sits before the rack reduction it wraps every
     * 2*PI/kHoodRackReduction radians of hood travel, exactly as on the real robot; angles outside
     * that window alias, which is why the code falls back on the relative encoder.
     */
    private void setSimAbsoluteEncoderAngle(double hoodAngleRad) {
        // Where the encoder physically reads zero, in hood radians, as decoded by the code.
        double zeroAngle =
                MathUtil.inputModulus(ShooterConstants.kHoodEncoderOffset, -Math.PI, Math.PI);
        double sign = ShooterConstants.kHoodAbsoluteEncoderInverted ? -1 : 1;
        double fraction =
                MathUtil.inputModulus(
                        sign
                                * (hoodAngleRad - zeroAngle)
                                * ShooterConstants.kHoodRackReduction
                                / (2 * Math.PI),
                        0.0,
                        1.0);
        absoluteEncoderSim.set(fraction);
    }
}
