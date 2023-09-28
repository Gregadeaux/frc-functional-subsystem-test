package frc.robot.subsystems.arm;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.arm.state.ArmGoalState;
import frc.robot.subsystems.arm.state.ArmInputState;
import frc.robot.subsystems.arm.state.ArmOutputState;

public class ArmDefaultBehavior implements ArmBehavior {
    private final ProfiledPIDController controller;
    private final ArmFeedforward ff;
    
    public ArmDefaultBehavior() {
        this.controller = new ProfiledPIDController(1, 0, 0, new Constraints(0.1, .1));
        this.controller.setTolerance(1, 1);
        this.controller.enableContinuousInput(0, 360);

        this.ff = new ArmFeedforward(0, 0, 0);
    }
    
    @Override
    public ArmOutputState periodic(ArmInputState inputs, ArmGoalState goal) {
        if (DriverStation.isEnabled()) {
            double angle = inputs.currentAngleDegrees();
            double velocity = inputs.velocityDegreesPerSecond();
            double effort = controller.calculate(angle, goal.position());
            double feedforward = ff.calculate(Units.degreesToRadians(angle), Units.degreesToRadians(velocity));

            effort += feedforward;
            effort = MathUtil.clamp(effort, -6, 6);
            return new ArmOutputState(Optional.of(effort), Optional.empty());
        }
        return new ArmOutputState(Optional.empty(), Optional.empty());
    }
}
