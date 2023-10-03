package frc.robot.subsystems.arm.state;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public record ArmOutputState(
    Optional<Double> voltage,
    Optional<Double> adjustOffsetDegrees
) implements LoggableInputs {

    private static final String KEY_VOLTAGE = "ArmSetVoltage";
    private static final String KEY_ADJUST_OFFSET = "ArmSetAdjustOffset";

    @Override
    public void toLog(LogTable table) {
        voltage.ifPresent((volts) -> table.put(KEY_VOLTAGE, volts));
        adjustOffsetDegrees.ifPresent((offset) -> table.put(KEY_ADJUST_OFFSET, offset));
    }

    @Override
    public void fromLog(LogTable table) { }
}
