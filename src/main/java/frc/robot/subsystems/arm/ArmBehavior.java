package frc.robot.subsystems.arm;

import frc.robot.subsystems.arm.state.ArmGoalState;
import frc.robot.subsystems.arm.state.ArmInputState;
import frc.robot.subsystems.arm.state.ArmOutputState;
import frc.robot.subsystems.common.Behavior;

public interface ArmBehavior extends Behavior<ArmInputState, ArmOutputState, ArmGoalState> {}
