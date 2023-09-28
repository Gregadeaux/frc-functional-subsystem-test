package frc.robot.subsystems.common;

import java.util.function.Consumer;
import java.util.function.Function;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public record LoggableFunction<T>(
    Consumer<LogTable> toLog, 
    Function<LogTable, T> fromLog, 
    Consumer<T> callback
) implements LoggableInputs{

    public interface LoggableRecord<T> {
        public void toLog(LogTable table);
        public T fromLog(LogTable table);
    }

    @Override
    public void toLog(LogTable table) {
        toLog.accept(table);
    }

    @Override
    public void fromLog(LogTable table) {
        callback.accept(fromLog.apply(table));
    }
}
