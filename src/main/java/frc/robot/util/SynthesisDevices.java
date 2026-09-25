package frc.robot.util;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;

/**
 * The three device types Autodesk Synthesis understands, published as WPILib {@link SimDevice}s.
 *
 * <p>Synthesis talks to robot code through WPILib's HALSim websocket server. It does not see REV or
 * CTRE objects; it sees SimDevices whose names start with {@code CANMotor:}, {@code CANEncoder:} or
 * {@code Gyro:} and whose fields follow the layout below. This is the same protocol Synthesis's own
 * SyntheSimJava wrappers publish (Apache-2.0, github.com/Autodesk/synthesis), written out here so
 * the robot needs no extra dependency and the real-robot classes stay free of simulation code.
 *
 * <p>Units, as Synthesis reports them:
 *
 * <ul>
 *   <li>Motor output is a fraction in [-1, 1]. Synthesis treats it as a <em>velocity</em> request:
 *       the joint is driven toward {@code output * maxVelocity}, where maxVelocity is set per joint
 *       in the Synthesis robot configuration. It is not a voltage.
 *   <li>Encoder position is radians and velocity radians per second, measured at the joint (the
 *       wheel or the steering hinge), so no gear ratio applies.
 *   <li>Gyro angles are degrees and rates degrees per second.
 * </ul>
 */
public final class SynthesisDevices {
    private SynthesisDevices() {}

    private static FieldPose fieldPose;

    /** The one field-pose reader; a SimDevice name can only be created once. */
    public static synchronized FieldPose fieldPose() {
        if (fieldPose == null) {
            fieldPose = new FieldPose();
        }
        return fieldPose;
    }

    /** A motor controller. Synthesis reads the output; everything else is informational. */
    public static final class Motor {
        private final SimDouble percentOutput;
        private final SimBoolean brakeMode;

        public Motor(String name, int canId) {
            SimDevice device = SimDevice.create("CANMotor:" + name, canId);
            device.createBoolean("init", Direction.kOutput, true);
            percentOutput = device.createDouble("percentOutput", Direction.kOutput, 0.0);
            brakeMode = device.createBoolean("brakeMode", Direction.kOutput, false);
            device.createDouble("neutralDeadband", Direction.kOutput, 0.0);
            device.createDouble("supplyCurrent", Direction.kInput, 0.0);
            device.createDouble("motorCurrent", Direction.kInput, 0.0);
            device.createDouble("busVoltage", Direction.kInput, 0.0);
        }

        public void set(double output) {
            if (!Double.isFinite(output)) output = 0.0;
            percentOutput.set(Math.max(-1.0, Math.min(1.0, output)));
        }

        public double get() {
            return percentOutput.get();
        }

        public void setBrakeMode(boolean brake) {
            brakeMode.set(brake);
        }
    }

    /** A joint encoder. Synthesis writes both fields every physics step. */
    public static final class Encoder {
        private final SimDouble position;
        private final SimDouble velocity;

        public Encoder(String name, int canId) {
            SimDevice device = SimDevice.create("CANEncoder:" + name, canId);
            device.createBoolean("init", Direction.kOutput, true);
            position = device.createDouble("position", Direction.kInput, 0.0);
            velocity = device.createDouble("velocity", Direction.kInput, 0.0);
        }

        /** Radians at the joint. */
        public double getPositionRad() {
            return position.get();
        }

        /** Radians per second at the joint. */
        public double getVelocityRadPerSec() {
            return velocity.get();
        }
    }

    /** A three-axis gyro mounted on the chassis. */
    public static final class Gyro {
        private final SimDouble angleX;
        private final SimDouble angleY;
        private final SimDouble angleZ;
        private final SimDouble rateX;
        private final SimDouble rateY;
        private final SimDouble rateZ;

        public Gyro(String name, int canId) {
            SimDevice device = SimDevice.create("Gyro:" + name, canId);
            device.createDouble("range", Direction.kOutput, 0.0);
            device.createBoolean("connected", Direction.kOutput, true);
            angleX = device.createDouble("angle_x", Direction.kInput, 0.0);
            angleY = device.createDouble("angle_y", Direction.kInput, 0.0);
            angleZ = device.createDouble("angle_z", Direction.kInput, 0.0);
            rateX = device.createDouble("rate_x", Direction.kInput, 0.0);
            rateY = device.createDouble("rate_y", Direction.kInput, 0.0);
            rateZ = device.createDouble("rate_z", Direction.kInput, 0.0);
        }

        /** Degrees about the chosen axis ('x', 'y' or 'z' in Synthesis's naming). */
        public double getAngleDeg(char axis) {
            return switch (axis) {
                case 'x' -> angleX.get();
                case 'y' -> angleY.get();
                default -> angleZ.get();
            };
        }

        /** Degrees per second about the chosen axis. */
        public double getRateDegPerSec(char axis) {
            return switch (axis) {
                case 'x' -> rateX.get();
                case 'y' -> rateY.get();
                default -> rateZ.get();
            };
        }
    }

    /**
     * The robot's true field pose, written by Synthesis each physics step.
     *
     * <p>Synthesis only writes to motor, encoder and gyro devices, so the pose rides on two encoder
     * channels: "FieldPoseXY" carries x in position and y in velocity (metres, WPILib field frame),
     * and "FieldPoseTheta" carries the heading (radians) in position and, in velocity, a counter
     * that Synthesis bumps whenever it places or teleports the robot.
     */
    public static final class FieldPose {
        private final Encoder xy = new Encoder("FieldPoseXY", 90);
        private final Encoder theta = new Encoder("FieldPoseTheta", 91);

        private FieldPose() {}

        public double x() {
            return xy.getPositionRad();
        }

        public double y() {
            return xy.getVelocityRadPerSec();
        }

        public double headingRad() {
            return theta.getPositionRad();
        }

        /** 0 until Synthesis has written a pose; increments on every placement. */
        public int placementCount() {
            return (int) Math.round(theta.getVelocityRadPerSec());
        }
    }
}
