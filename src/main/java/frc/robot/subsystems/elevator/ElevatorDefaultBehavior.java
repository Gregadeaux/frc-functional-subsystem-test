package frc.robot.subsystems.elevator;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.common.Behavior;
import frc.robot.subsystems.elevator.ElevatorState.ElevatorGoalState;
import frc.robot.subsystems.elevator.ElevatorState.ElevatorInputState;
import frc.robot.subsystems.elevator.ElevatorState.ElevatorMetadata;
import frc.robot.subsystems.elevator.ElevatorState.ElevatorOutputState;

public class ElevatorDefaultBehavior extends Behavior<ElevatorInputState, ElevatorOutputState, ElevatorGoalState, ElevatorMetadata> {
    private final ProfiledPIDController controller;
    private final ElevatorFeedforward ff;

    public ElevatorDefaultBehavior() {
        this.controller = new ProfiledPIDController(45, 0, 0, 
                 new Constraints(Units.inchesToMeters(90), Units.inchesToMeters(90))); //This is in meters

        this.ff = new ElevatorFeedforward(0, 0.45, 0, 0);
        setState(new ElevatorMetadata(this.controller));
    }

    @Override
    public ElevatorOutputState periodic(ElevatorInputState inputs, ElevatorGoalState goal, ElevatorMetadata metadata) {
        if (DriverStation.isEnabled()) {
            Optional<Double> voltage = Optional.of(inputs)
                .map( i -> new double[] { 
                    controller.calculate(i.position(), goal.position()),
                    ff.calculate(i.velocity())
                }).map(tuple -> tuple[0] + tuple[1])
                .map(effort -> MathUtil.clamp(effort, -12, 12));

            setState(new ElevatorMetadata(this.controller));
            return new ElevatorOutputState(voltage);
        }

        return new ElevatorOutputState(Optional.empty());
    }
    
}
