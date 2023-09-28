package frc.robot.subsystems.arm.state;

import org.littletonrobotics.junction.LogTable;

import frc.robot.subsystems.common.LoggableFunction.LoggableRecord;

public record ArmGoalState(double position) implements LoggableRecord<ArmGoalState> {

    private static final String KEY_POSITION = "ArmGoalPosition";

    @Override
    public void toLog(LogTable table) {
        table.put(KEY_POSITION, position);
    }

    @Override
    public ArmGoalState fromLog(LogTable table) {
        return new ArmGoalState(
            table.getDouble(KEY_POSITION, position)
        );
    } 
}
