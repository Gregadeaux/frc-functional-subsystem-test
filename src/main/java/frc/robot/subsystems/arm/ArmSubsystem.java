package frc.robot.subsystems.arm;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.hardware.ArmHardwareLayerInterface;
import frc.robot.subsystems.arm.state.ArmGoalState;
import frc.robot.subsystems.arm.state.ArmInputState;
import frc.robot.subsystems.arm.state.ArmOutputState;
import frc.robot.subsystems.common.BearsSubsystemBase;

public class ArmSubsystem extends BearsSubsystemBase<ArmInputState, ArmOutputState, ArmGoalState> {
    public ArmSubsystem(ArmHardwareLayerInterface armIO, ArmBehavior business) {
        super(armIO::getState, armIO::setState, business, new ArmGoalState(45));
    }

    public Command setArmPositionCommand(double degrees) {
        return new InstantCommand(() -> setGoal(new ArmGoalState(degrees)), this);
    }
}
