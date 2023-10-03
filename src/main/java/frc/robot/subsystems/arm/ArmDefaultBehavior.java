package frc.robot.subsystems.arm;

import java.util.Optional;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.arm.state.ArmGoalState;
import frc.robot.subsystems.arm.state.ArmInputState;
import frc.robot.subsystems.arm.state.ArmOutputState;
import frc.robot.subsystems.common.PidFf.*;

public class ArmDefaultBehavior extends ArmPidFfBehavior<ArmInputState, ArmOutputState, ArmGoalState> {
    public ArmDefaultBehavior() {
        super(
            new ProfiledPIDController(1, 0, 0, new Constraints(0.1, .1)),
            new ArmFeedforward(0, 0, 0),
            (PidFfOutput pidOut) -> new ArmOutputState(pidOut.effortVoltage(), Optional.empty())
        );
    }
}
