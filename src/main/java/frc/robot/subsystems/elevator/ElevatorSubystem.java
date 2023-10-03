package frc.robot.subsystems.elevator;

import frc.robot.subsystems.common.BearsSubsystemBase;
import frc.robot.subsystems.common.Behavior;
import frc.robot.subsystems.elevator.ElevatorState.ElevatorGoalState;
import frc.robot.subsystems.elevator.ElevatorState.ElevatorInputState;
import frc.robot.subsystems.elevator.ElevatorState.ElevatorMetadata;
import frc.robot.subsystems.elevator.ElevatorState.ElevatorOutputState;
import frc.robot.subsystems.elevator.hardware.ElevatorHardwareInterface;

public class ElevatorSubystem extends BearsSubsystemBase<ElevatorInputState, ElevatorOutputState, ElevatorGoalState> {

    public ElevatorSubystem(ElevatorHardwareInterface elevatorIO, Behavior<ElevatorInputState, ElevatorOutputState, ElevatorGoalState, ElevatorMetadata> behavior) {
        super(elevatorIO::getState, elevatorIO::setState, behavior, new ElevatorGoalState(0));
    }
    
}
