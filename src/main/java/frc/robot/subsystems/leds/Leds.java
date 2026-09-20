package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.Alerts;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * The LED strip.
 *
 * <p>This is now an ordinary subsystem that <em>reads</em> {@link RobotState} and writes pixels.
 * Previously it was a singleton with public mutable fields that half the robot wrote into and that
 * the drivetrain and shooter both read back out, so the LED class was on the dependency path of
 * every mechanism: {@code Swerve.getMaxSpeed()} asked the light strip whether demo mode was on.
 *
 * <p>It is also no longer driven from {@code Robot.robotPeriodic()} by hand. Being a subsystem, the
 * command scheduler runs it, which means it participates in the same logging and timing as
 * everything else.
 */
public class Leds extends SubsystemBase {
    private static final int kLength = 42;
    private static final int kBottomLength = 8;
    private static final int kLedPort = 7;
    private static final int kMinLoopCycles = 10;
    private static final double kLowBatteryVolts = 10.0;

    private static final double kShimmerExtremeness = 0.5;
    private static final double kShimmerSpeed = 1.0;
    private static final double kStrobeTickSkip = 15.0;
    private static final int kStrobeDuration = 5;
    private static final double kWaveExponent = 0.4;
    private static final double kWaveCycleLength = 25.0;
    private static final double kWaveDuration = 3.0;
    private static final double kBreathDuration = 1.0;

    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private final Notifier bootAnimation;
    private boolean bootAnimationStopped = false;

    private int loopCycleCount = 0;

    public Leds() {
        leds = new AddressableLED(kLedPort);
        buffer = new AddressableLEDBuffer(kLength);
        leds.setLength(kLength);

        if (Constants.lightsExist) {
            leds.setData(buffer);
            leds.start();
        }

        // Breathe purple on a Notifier while the rest of the robot is still constructing, so the
        // strip shows something during the several seconds before the scheduler starts running.
        bootAnimation =
                new Notifier(
                        () -> {
                            synchronized (this) {
                                breath(
                                        Section.FULL,
                                        Color.kPurple,
                                        Color.kBlack,
                                        0.4,
                                        System.currentTimeMillis() / 1000.0);
                                leds.setData(buffer);
                            }
                        });
        bootAnimation.startPeriodic(Constants.kLoopPeriodSecs);
    }

    @Override
    public void periodic() {
        if (!bootAnimationStopped) {
            bootAnimation.stop();
            bootAnimation.close();
            bootAnimationStopped = true;
        }

        RobotState state = RobotState.getInstance();
        state.setLowBattery(RobotController.getBatteryVoltage() < kLowBatteryVolts);

        if (!Constants.lightsExist) {
            return;
        }

        // The strip flickers while the roboRIO is still bringing up its PWM output.
        if (++loopCycleCount < kMinLoopCycles) {
            return;
        }

        synchronized (this) {
            solid(Section.FULL, Color.kBlack);

            switch (state.getMode()) {
                case DISABLED -> shimmer(Section.FULL, Color.kOrange);
                    // The 4-argument overload reads the live FPGA clock. The 5-argument one takes
                    // a TIMESTAMP, not a speed; passing a constant to it froze the animation on a
                    // single colour for the whole of autonomous.
                case AUTO -> breath(Section.FULL, Color.kRed, Color.kBlue, 4);
                default -> teleopPattern(state);
            }

            // An error condition overrides everything: the drive team needs to know before the
            // match starts, not after.
            if (Alerts.hasType(edu.wpi.first.wpilibj.Alert.AlertType.kError)) {
                strobe(Section.FULL, Color.kRed);
            } else if (state.isLowBattery()) {
                strobe(Section.BOTTOM, Color.kMagenta);
            }

            leds.setData(buffer);
        }

        Logger.recordOutput("Leds/LoopCycles", loopCycleCount);
    }

    private void teleopPattern(RobotState state) {
        Color primary;
        if (state.getAlliance() == Alliance.Blue) {
            wave(Section.FULL, Color.kLightBlue, Color.kDarkBlue, kWaveCycleLength, kWaveDuration);
            primary = Color.kBlue;
        } else {
            wave(Section.FULL, Color.kFirstRed, Color.kRed, kWaveCycleLength, kWaveDuration);
            primary = Color.kRed;
        }

        if (state.isIntakeDeployed()) {
            stripes(Section.FULL, List.of(primary, Color.kPurple), 4, 1);
        }
        if (state.areWheelsLocked()) {
            stripes(Section.FULL, List.of(primary, Color.kOrange), 4, 0.5);
        }
        if (state.isShooterAtSpeed()) {
            strobe(Section.FULL, Color.kGreen);
        }
        if (state.isDemoMode()) {
            rainbow(Section.FULL);
        }
    }

    // ------------------------------------------------------------------ patterns

    private void solid(Section section, Color color) {
        for (int i = section.start(); i < section.end(); i++) {
            buffer.setLED(i, color);
        }
    }

    private void shimmer(Section section, Color color) {
        for (int i = section.start(); i < section.end(); i++) {
            double brightness =
                    kShimmerExtremeness + Math.sin((loopCycleCount + i) * 0.01) * kShimmerSpeed;
            buffer.setLED(
                    i,
                    new Color(
                            color.red * brightness,
                            color.green * brightness,
                            color.blue * brightness));
        }
    }

    private void rainbow(Section section) {
        for (int i = section.start(); i < section.end(); i++) {
            int hue = ((loopCycleCount * 3) % 180 + (i * 180 / kLength)) % 180;
            buffer.setHSV(i, hue, 255, 128);
        }
    }

    private void strobe(Section section, Color color) {
        boolean on = loopCycleCount % kStrobeTickSkip < kStrobeDuration;
        for (int i = section.start(); i < section.end(); i++) {
            if (on) {
                buffer.setLED(i, color);
            } else {
                buffer.setHSV(i, 0, 0, 0);
            }
        }
    }

    private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
        double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (int i = 0; i < section.end(); i++) {
            x += xDiffPerLed;
            if (i >= section.start()) {
                double ratio = (Math.pow(Math.sin(x), kWaveExponent) + 1.0) / 2.0;
                if (Double.isNaN(ratio)) {
                    ratio = (-Math.pow(Math.sin(x + Math.PI), kWaveExponent) + 1.0) / 2.0;
                }
                if (Double.isNaN(ratio)) {
                    ratio = 0.5;
                }
                buffer.setLED(i, lerp(c1, c2, ratio));
            }
        }
    }

    private void stripes(Section section, List<Color> colors, int frontLength, double duration) {
        int offset =
                (int)
                        (Timer.getFPGATimestamp()
                                % duration
                                / duration
                                * frontLength
                                * colors.size());
        for (int i = section.start(); i < section.end(); i++) {
            int index =
                    (int) (Math.floor((double) (i - offset) / frontLength) + colors.size())
                            % colors.size();
            buffer.setLED(i, colors.get(colors.size() - 1 - index));
        }
    }

    private void breath(Section section, Color c1, Color c2, double duration) {
        breath(section, c1, c2, duration, Timer.getFPGATimestamp());
    }

    private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
        double x = ((timestamp % kBreathDuration) / kBreathDuration) * 2.0 * Math.PI;
        solid(section, lerp(c1, c2, (Math.sin(x) + 1.0) / 2.0));
    }

    private static Color lerp(Color a, Color b, double ratio) {
        return new Color(
                a.red * (1 - ratio) + b.red * ratio,
                a.green * (1 - ratio) + b.green * ratio,
                a.blue * (1 - ratio) + b.blue * ratio);
    }

    private static enum Section {
        FULL,
        BOTTOM,
        UPPER;

        private int start() {
            return this == UPPER ? kBottomLength : 0;
        }

        private int end() {
            return this == BOTTOM ? kBottomLength : kLength;
        }
    }
}
