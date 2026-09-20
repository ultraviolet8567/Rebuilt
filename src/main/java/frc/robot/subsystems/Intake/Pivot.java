package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.subsystems.Lights;
import frc.robot.util.SimBattery;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Pivot extends SubsystemBase {
    private final SparkMax pivotMotor; // Spark Max Raises
    private final SparkMaxConfig pivotMotorConfig;
    private final RelativeEncoder relativeEncoder;
    private final DutyCycleEncoder absoluteEncoder;

    private final PIDController pidController;
    private final ArmFeedforward feedforwardController;

    private final GenericEntry pivotDisplay;

    private double targetPosition;

    // Total reduction from motor shaft to pivot angle.
    private static final double kTotalReduction =
            IntakeConstants.kPivotGearboxReduction * IntakeConstants.kPivotChainReduction;

    // Mechanism2d drawing for AdvantageScope (real robot and sim).
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
    private final LoggedMechanismLigament2d pivotLigament;

    // Desktop simulation only (null on the real robot).
    private final SingleJointedArmSim pivotSim;
    private final SparkMaxSim pivotSparkSim;
    private final DutyCycleEncoderSim absoluteEncoderSim;

    public Pivot() {
        System.out.println("[Init] Creating Pivot");

        pivotMotor = new SparkMax(CAN.kPivotPort, MotorType.kBrushless);
        pivotMotorConfig = new SparkMaxConfig();
        relativeEncoder = pivotMotor.getEncoder();
        pivotMotorConfig.encoder.positionConversionFactor(1.0 / kTotalReduction);
        pivotMotorConfig.smartCurrentLimit(50);
        pivotMotor.configure(
                pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        absoluteEncoder = new DutyCycleEncoder(CAN.kPivotEncoderPort);

        pidController =
                new PIDController(
                        IntakeConstants.kPivotP.get(),
                        IntakeConstants.kPivotI.get(),
                        IntakeConstants.kPivotD.get());
        feedforwardController =
                new ArmFeedforward(
                        IntakeConstants.kPivotS.get(),
                        IntakeConstants.kPivotG.get(),
                        IntakeConstants.kPivotV.get(),
                        IntakeConstants.kPivotA.get());

        LoggedMechanismRoot2d root = mechanism.getRoot("IntakePivot", 0.7, 0.3);
        pivotLigament =
                root.append(
                        new LoggedMechanismLigament2d(
                                "Intake",
                                SimConstants.kPivotArmLengthMeters,
                                90,
                                6,
                                new Color8Bit(Color.kPurple)));

        if (RobotBase.isSimulation()) {
            DCMotor gearbox = DCMotor.getNEO(1);
            // The team's ArmFeedforward is fed (pivot angle + kPivotFeedforwardOffset) as the
            // angle at which gravity acts, so the physics model uses that same frame: sim angle
            // 0 is horizontal, gravity pulls toward -PI/2.
            double startSim = pivotAngleToSimAngle(IntakeConstants.kPivotLower);
            pivotSim =
                    new SingleJointedArmSim(
                            gearbox,
                            kTotalReduction,
                            SingleJointedArmSim.estimateMOI(
                                    SimConstants.kPivotArmLengthMeters,
                                    SimConstants.kPivotArmMassKg),
                            SimConstants.kPivotArmLengthMeters,
                            pivotAngleToSimAngle(
                                    IntakeConstants.kPivotLower
                                            - SimConstants.kPivotHardStopMarginRads),
                            pivotAngleToSimAngle(
                                    IntakeConstants.kPivotUpper
                                            + SimConstants.kPivotHardStopMarginRads),
                            true,
                            startSim);
            pivotSparkSim = new SparkMaxSim(pivotMotor, gearbox);
            absoluteEncoderSim = new DutyCycleEncoderSim(absoluteEncoder);
            absoluteEncoderSim.setConnected(true);

            // Boot stowed (kPivotLower is "up"), like a robot that was powered off stowed.
            setSimAbsoluteEncoderAngle(IntakeConstants.kPivotLower);
        } else {
            pivotSim = null;
            pivotSparkSim = null;
            absoluteEncoderSim = null;
        }

        resetRelativeEncoder();

        targetPosition = getAbsoluteRotationRads();

        pivotDisplay =
                Shuffleboard.getTab("Main")
                        .add("Pivot Up?", atPosition(IntakeConstants.kPivotLower))
                        .withWidget(BuiltInWidgets.kBooleanBox)
                        .getEntry();
    }

    public void periodic() {
        Logger.recordOutput("Intake/Pivot/AbsoluteRotation", getAbsoluteRotationRads());
        Logger.recordOutput("Intake/Pivot/RelativeRotation", getRelativeRotationRads());
        Logger.recordOutput(
                "Intake/Pivot/FFRotation",
                getRelativeRotationRads() + IntakeConstants.kPivotFeedforwardOffset);

        Logger.recordOutput("Intake/Pivot/Gains/pidP", pidController.getP());
        Logger.recordOutput("Intake/Pivot/Gains/pidI", pidController.getI());
        Logger.recordOutput("Intake/Pivot/Gains/pidD", pidController.getD());
        Logger.recordOutput("Intake/Pivot/Gains/ffS", feedforwardController.getKs());
        Logger.recordOutput("Intake/Pivot/Gains/ffG", feedforwardController.getKg());
        Logger.recordOutput("Intake/Pivot/Gains/ffV", feedforwardController.getKv());
        Logger.recordOutput("Intake/Pivot/Gains/ffA", feedforwardController.getKa());

        pidController.setP(IntakeConstants.kPivotP.get());
        pidController.setI(IntakeConstants.kPivotI.get());
        pidController.setD(IntakeConstants.kPivotD.get());
        feedforwardController.setKs(IntakeConstants.kPivotS.get());
        feedforwardController.setKg(IntakeConstants.kPivotG.get());
        feedforwardController.setKv(IntakeConstants.kPivotV.get());
        feedforwardController.setKa(IntakeConstants.kPivotA.get());

        double diff = Math.abs(getAbsoluteRotationRads() - getRelativeRotationRads());
        if (diff > 0.01 && diff < 0.1) resetRelativeEncoder();

        setAngleRads(targetPosition);
        pivotDisplay.setBoolean(atPosition(IntakeConstants.kPivotLower));
        Lights.getInstance().pivotDown = !atPosition(IntakeConstants.kPivotLower);

        // kPivotLower (0.1 rad) is stowed/up, kPivotUpper (2 rad) is deployed/down.
        pivotLigament.setAngle(90 - Units.radiansToDegrees(getRelativeRotationRads()));
        Logger.recordOutput("Mechanism2d/IntakePivot", mechanism);
    }

    public double getAbsoluteRotationRads() {
        double angle = absoluteEncoder.get();
        angle *= 2 * Math.PI;
        angle *= IntakeConstants.kPivotAbsoluteEncoderInverted ? -1 : 1;
        angle /= IntakeConstants.kPivotChainReduction;
        angle += IntakeConstants.kPivotEncoderOffset;
        angle = MathUtil.inputModulus(angle, -Math.PI * 2, Math.PI);
        return angle;
    }

    public double getRelativeRotationRads() {
        double angle = relativeEncoder.getPosition();
        angle *= 2 * Math.PI * (IntakeConstants.kPivotRelativeEncoderInverted ? -1 : 1);
        return angle;
    }

    public void resetRelativeEncoder() {
        double angle = getAbsoluteRotationRads();
        angle /= 2 * Math.PI;
        angle *= (IntakeConstants.kPivotRelativeEncoderInverted ? -1 : 1);
        relativeEncoder.setPosition(angle);
        // REV's simulated Spark keeps its own copy of the position and overwrites the encoder
        // with it every loop, so in simulation the sim object has to be told as well.
        if (pivotSparkSim != null) pivotSparkSim.setPosition(angle);
    }

    public boolean atPosition(double position) {
        return Math.abs(position - getRelativeRotationRads()) < IntakeConstants.kPivotTolerance;
    }

    public void setAngleRads(double angle) {
        double pidVoltage =
                pidController.calculate(
                        getRelativeRotationRads(),
                        MathUtil.clamp(
                                angle, IntakeConstants.kPivotLower, IntakeConstants.kPivotUpper));
        double ffVoltage =
                feedforwardController.calculate(
                        getRelativeRotationRads() + IntakeConstants.kPivotFeedforwardOffset, 0);

        Logger.recordOutput("Intake/Pivot/TargetRotation", angle);
        Logger.recordOutput("Intake/Pivot/pidVoltage", pidVoltage);
        Logger.recordOutput("Intake/Pivot/ffVoltage", ffVoltage);

        setPivotVoltage(pidVoltage + ffVoltage);
    }

    public void setPivotVoltage(double voltage) {
        voltage =
                MathUtil.clamp(
                        voltage, -IntakeConstants.kPivotVoltage, IntakeConstants.kPivotVoltage);
        voltage *= IntakeConstants.kPivotInverted ? -1 : 1;
        pivotMotor.setVoltage(voltage);
    }

    public void stop() {
        setPivotVoltage(0);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition =
                MathUtil.clamp(
                        targetPosition, IntakeConstants.kPivotLower, IntakeConstants.kPivotUpper);
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    // ------------------------------------------------------------------------------------
    // Desktop simulation
    // ------------------------------------------------------------------------------------

    /**
     * Convert between the pivot angle the code uses and the arm-sim angle. The relative encoder
     * counts positive for positive applied voltage; getRelativeRotationRads() applies the
     * kPivotRelativeEncoderInverted sign; and the gravity frame is offset by
     * kPivotFeedforwardOffset (matching the team's ArmFeedforward usage).
     */
    private static double pivotAngleToSimAngle(double pivotAngleRad) {
        return pivotAngleRad * (IntakeConstants.kPivotRelativeEncoderInverted ? -1 : 1)
                + IntakeConstants.kPivotFeedforwardOffset;
    }

    private static double simAngleToPivotAngle(double simAngleRad) {
        return (simAngleRad - IntakeConstants.kPivotFeedforwardOffset)
                * (IntakeConstants.kPivotRelativeEncoderInverted ? -1 : 1);
    }

    @Override
    public void simulationPeriodic() {
        double dt = Constants.kLoopPeriodSecs;
        double vbus = RoboRioSim.getVInVoltage();

        pivotSim.setInputVoltage(pivotSparkSim.getAppliedOutput() * vbus);
        pivotSim.update(dt);

        // Relative encoder velocity factor is the default (motor RPM); position factor is
        // 1/kTotalReduction, so REV's sim integrates position in output rotations for us.
        double motorRpm =
                Units.radiansPerSecondToRotationsPerMinute(pivotSim.getVelocityRadPerSec())
                        * kTotalReduction;
        pivotSparkSim.iterate(motorRpm, vbus, dt);

        setSimAbsoluteEncoderAngle(simAngleToPivotAngle(pivotSim.getAngleRads()));

        SimBattery.addCurrent(pivotSim.getCurrentDrawAmps());
    }

    /**
     * Set the duty-cycle absolute encoder so getAbsoluteRotationRads() returns the given pivot
     * angle. The encoder sits before the chain reduction so it wraps every
     * 2*PI/kPivotChainReduction radians of pivot travel, exactly as on the real robot. Note that
     * with the current offset the readable window is roughly (-1.9, 0.62] rad: the stowed position
     * is inside it, the deployed position (2 rad) is not and aliases, which is why the code only
     * re-syncs the relative encoder when the two agree to within 0.1 rad.
     */
    private void setSimAbsoluteEncoderAngle(double pivotAngleRad) {
        double zeroAngle =
                MathUtil.inputModulus(IntakeConstants.kPivotEncoderOffset, -Math.PI * 2, Math.PI);
        double sign = IntakeConstants.kPivotAbsoluteEncoderInverted ? -1 : 1;
        double fraction =
                MathUtil.inputModulus(
                        sign
                                * (pivotAngleRad - zeroAngle)
                                * IntakeConstants.kPivotChainReduction
                                / (2 * Math.PI),
                        0.0,
                        1.0);
        absoluteEncoderSim.set(fraction);
    }
}
