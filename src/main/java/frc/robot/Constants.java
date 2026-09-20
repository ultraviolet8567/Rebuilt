// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Global, robot-wide switches. Everything mechanism-specific now lives next to the mechanism, in a
 * per-package constants class, so that a subsystem and its numbers can be read, moved or deleted as
 * one unit -- the arrangement used by 41 of the 42 Java robots in the 2026 reference corpus, and by
 * all of the ones that split hardware behind an IO layer.
 */
public final class Constants {
    private Constants() {}

    /**
     * Where the code is running.
     *
     * <p>REPLAY re-runs a previously recorded WPILOG through the exact same subsystem code with the
     * real hardware replaced by the log's recorded inputs. It is the reason the IO layer exists:
     * every subsystem reads its sensors through an interface whose values are logged, so the log
     * can be played back as if it were the robot. 19 of the 42 corpus robots carry this mode.
     */
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    /** Set to REPLAY by hand (and re-run in simulation) to replay a log. */
    private static final Mode kSimMode = Mode.SIM;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

    /** Publishes tunable numbers to NetworkTables. Turn OFF for competition. */
    public static final boolean tuningMode = true;

    /** False when the LED strip is not plugged in, so the code does not drive a missing device. */
    public static final boolean lightsExist = true;

    /** Main robot loop period, seconds. */
    public static final double kLoopPeriodSecs = 0.02;

    public static final class OIConstants {
        private OIConstants() {}

        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        /**
         * Applied to the joystick VECTOR magnitude, not to each axis. See {@link
         * frc.robot.commands.DriveCommands}.
         */
        public static final double kDeadband = 0.11;
    }
}
