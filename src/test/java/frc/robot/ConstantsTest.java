package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.ShooterConstants;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.LinkedHashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

/**
 * Sanity checks on {@link Constants} that do not need hardware.
 *
 * <p>These catch the mistakes that are expensive on a real robot: two devices on the same CAN ID, a
 * CAN ID outside the legal range, or a mechanism limit configured backwards.
 */
class ConstantsTest {
    /**
     * Every {@code int} in {@link Constants.CAN} that is a CAN device ID.
     *
     * <p>Encoder channels are excluded: the hood and pivot absolute encoders are {@code
     * DutyCycleEncoder} channels on the roboRIO, not CAN devices, so their numbers may legitimately
     * overlap with CAN IDs.
     */
    private static Map<String, Integer> canDeviceIds() throws IllegalAccessException {
        Map<String, Integer> ids = new LinkedHashMap<>();
        for (Field field : Constants.CAN.class.getDeclaredFields()) {
            if (!Modifier.isStatic(field.getModifiers()) || field.getType() != int.class) {
                continue;
            }
            if (field.getName().contains("Encoder")) {
                continue;
            }
            ids.put(field.getName(), field.getInt(null));
        }
        return ids;
    }

    @Test
    void canDeviceIdsAreUnique() throws IllegalAccessException {
        Map<String, Integer> ids = canDeviceIds();
        assertFalse(ids.isEmpty(), "No CAN device IDs found in Constants.CAN");

        Map<Integer, String> owners = new LinkedHashMap<>();
        for (Map.Entry<String, Integer> entry : ids.entrySet()) {
            String existing = owners.put(entry.getValue(), entry.getKey());
            assertEquals(
                    null,
                    existing,
                    "CAN ID "
                            + entry.getValue()
                            + " is used by both "
                            + existing
                            + " and "
                            + entry.getKey());
        }
    }

    @Test
    void canDeviceIdsAreInLegalRange() throws IllegalAccessException {
        for (Map.Entry<String, Integer> entry : canDeviceIds().entrySet()) {
            int id = entry.getValue();
            assertTrue(
                    id >= 1 && id <= 62,
                    entry.getKey() + " = " + id + " is outside the legal FRC CAN range 1-62");
        }
    }

    @Test
    void controllerPortsDiffer() {
        assertNotEquals(
                Constants.OIConstants.kDriverControllerPort,
                Constants.OIConstants.kOperatorControllerPort,
                "Driver and operator controllers cannot share a port");
    }

    @Test
    void mechanismVoltagesStayUnderTwelveVolts() {
        assertPositiveAndSane("kFlywheelVoltage", ShooterConstants.kFlywheelVoltage);
        assertPositiveAndSane("kKickerVoltage", ShooterConstants.kKickerVoltage);
        assertPositiveAndSane("kHoodVoltage", ShooterConstants.kHoodVoltage);
        assertPositiveAndSane("kPivotVoltage", Constants.IntakeConstants.kPivotVoltage);
        assertPositiveAndSane("kFunnelVoltage", Constants.IntakeConstants.kFunnelVoltage);
        assertPositiveAndSane("kIndexerVoltage", Constants.StorageConstants.kIndexerVoltage);
    }

    private static void assertPositiveAndSane(String name, double voltage) {
        assertTrue(voltage > 0, name + " must be positive, got " + voltage);
        assertTrue(voltage <= 12, name + " must not exceed 12 V, got " + voltage);
    }

    @Test
    void setpointRangesAreOrdered() {
        assertTrue(
                ShooterConstants.kHoodLower < ShooterConstants.kHoodUpper,
                "Hood lower limit must be below its upper limit");
        assertTrue(
                Constants.IntakeConstants.kPivotLower < Constants.IntakeConstants.kPivotMiddle,
                "Pivot lower limit must be below the middle setpoint");
        assertTrue(
                Constants.IntakeConstants.kPivotMiddle < Constants.IntakeConstants.kPivotUpper,
                "Pivot middle setpoint must be below the upper setpoint");
    }

    @Test
    void demoScaleFactorsAreBetweenZeroAndOne() {
        assertTrue(
                ShooterConstants.shooterDemoScaleFactor > 0
                        && ShooterConstants.shooterDemoScaleFactor < 1,
                "Demo shooter scale must be a fraction, got "
                        + ShooterConstants.shooterDemoScaleFactor);
        assertTrue(
                Constants.DriveConstants.swerveDemoScaleFactor > 0
                        && Constants.DriveConstants.swerveDemoScaleFactor < 1,
                "Demo swerve scale must be a fraction, got "
                        + Constants.DriveConstants.swerveDemoScaleFactor);
    }
}
