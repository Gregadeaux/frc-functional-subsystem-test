package frc.robot.subsystems.arm.state;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public record ArmInputState(
    double currentAngleDegrees,
    double velocityDegreesPerSecond
) implements LoggableInputs {

    private static final String KEY_CURRENT_ANGLE_DEGREES = "ArmCurrentAngleDegrees";
    private static final String KEY_VELOCITY_DEGREES_PER_SECOND = "ArmVelocityDegrees";

    @Override
    public void toLog(LogTable table) {
        table.put(KEY_CURRENT_ANGLE_DEGREES, currentAngleDegrees);
        table.put(KEY_VELOCITY_DEGREES_PER_SECOND, velocityDegreesPerSecond);
    }

    @Override
    public void fromLog(LogTable table) {} 
}
