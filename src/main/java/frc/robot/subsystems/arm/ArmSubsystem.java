package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.hardware.ArmHardwareLayerInterface;
import frc.robot.subsystems.arm.state.ArmGoalState;
import frc.robot.subsystems.arm.state.ArmInputState;
import frc.robot.subsystems.arm.state.ArmOutputState;
import frc.robot.subsystems.common.BearsSubsystemBase;
import frc.robot.subsystems.common.Behavior;
import frc.robot.subsystems.common.PidFf.ArmPidFfMetadata;

public class ArmSubsystem extends BearsSubsystemBase<ArmInputState, ArmOutputState, ArmGoalState> {

    private static record Metadata(boolean atSetpoint) {}

    private Metadata metadata = new Metadata(false);

    public ArmSubsystem(ArmHardwareLayerInterface armIO, Behavior<ArmInputState, ArmOutputState, ArmGoalState, ArmPidFfMetadata> business) {
        super(armIO::getState, armIO::setState, business, new ArmGoalState(45));
        business.setStateConsumer(this::mapMetadata);
    }

    private void mapMetadata(ArmPidFfMetadata state) {
        this.metadata = new Metadata(
            state.pidController().atSetpoint()
        );
    }

    public Command setArmPositionCommand(double degrees) {
        return new InstantCommand(() -> setGoal(new ArmGoalState(degrees)), this);
    }

    public Command waitUntilAtAngleCommand() {
        return Commands.waitUntil(() -> this.metadata.atSetpoint());
    }
}
