package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Helpers for talking to REV Spark controllers over CAN without silently believing bad data.
 *
 * <p>A Spark read that fails -- because the controller browned out, the CAN wire came loose, or the
 * frame simply has not arrived yet -- does not throw. It returns a stale or zero value and sets an
 * error code that nothing checks. A swerve module whose turn encoder quietly returns 0.0 will
 * confidently steer the wheel to "straight ahead" and the robot will drive into a wall.
 *
 * <p>{@link #ifOk} only hands the value to the caller when the Spark reports {@code kOk}, and
 * otherwise raises {@link #sparkStickyFault} so the subsystem can mark the device disconnected and
 * raise a driver-visible {@link edu.wpi.first.wpilibj.Alert}. {@link #tryUntilOk} retries
 * configuration, which routinely fails once at boot while the CAN bus is still enumerating.
 *
 * <p>Pattern taken from FRC 6328's AdvantageKit template as vendored by FRC 6002 (2026 TriShooter)
 * and FRC 5406; 22 of the 42 corpus robots debounce device connection this way.
 */
public final class SparkUtil {
    private SparkUtil() {}

    /** Set by {@link #ifOk}/{@link #tryUntilOk} when a Spark read or write reported an error. */
    public static boolean sparkStickyFault = false;

    /** Hands {@code supplier}'s value to {@code consumer} only if the Spark read succeeded. */
    public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            consumer.accept(value);
        } else {
            sparkStickyFault = true;
        }
    }

    /** Same as {@link #ifOk}, for a group of reads that must all succeed together. */
    public static void ifOk(
            SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
        double[] values = new double[suppliers.length];
        for (int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if (spark.getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return;
            }
        }
        consumer.accept(values);
    }

    /** Runs a Spark command up to {@code maxAttempts} times, stopping at the first success. */
    public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
        for (int i = 0; i < maxAttempts; i++) {
            if (command.get() == REVLibError.kOk) {
                return;
            }
            sparkStickyFault = true;
        }
    }
}
