package frc.robot.subsystems.elevator;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.subsystems.common.LoggableFunction.LoggableRecord;

public class ElevatorState {
    public record ElevatorInputState(double velocity, double position) implements LoggableInputs {

        private static final String KEY_VELOCITY = "ElevatorInputVelocity";
        private static final String KEY_POSITION = "ElevatorInputPosition";

        @Override
        public void toLog(LogTable table) {
            table.put(KEY_VELOCITY, velocity);
            table.put(KEY_POSITION, position);  
        }

        @Override
        public void fromLog(LogTable table) {}
        
    }

    public record ElevatorOutputState(Optional<Double> voltage) implements LoggableInputs {

        private static final String KEY_VOLTAGE = "ElevatorOutputVoltage";

        @Override
        public void toLog(LogTable table) {
            voltage.ifPresent(v -> table.put(KEY_VOLTAGE, v));
        }

        @Override
        public void fromLog(LogTable table) {}

    }

    public record ElevatorGoalState(double position) implements LoggableRecord<ElevatorGoalState> {
        private static final String KEY_POSITION = "ElevatorGoalPosition";

        @Override
        public void toLog(LogTable table) {
            table.put(KEY_POSITION, position);
        }

        @Override
        public ElevatorGoalState fromLog(LogTable table) {
            return new ElevatorGoalState(
                table.getDouble(KEY_POSITION, position)
            );
        }
        
    }

    public record ElevatorMetadata(ProfiledPIDController pidController){}
}