package frc.robot.subsystems.common;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.common.LoggableFunction.LoggableRecord;

public abstract class BearsSubsystemBase<IN extends LoggableInputs, OUT extends LoggableInputs, GOAL extends LoggableRecord<GOAL>> extends SubsystemBase {

    private final Supplier<IN> input;
    private final Consumer<OUT> output;
    private final Behavior<IN, OUT, GOAL, ?> behavior;
    private GOAL goalState;
    private LoggableFunction<GOAL> loggableGoal;
    private IN currentState;

    private final String GOAL_LOG = this.getClass().getSimpleName() + "/goal";
    private final String INPUT_LOG = this.getClass().getSimpleName() + "/input";
    private final String OUTPUT_LOG = this.getClass().getSimpleName() + "/output";

    public BearsSubsystemBase(Supplier<IN> input, Consumer<OUT> output, Behavior<IN, OUT, GOAL, ?> behavior, GOAL goalState){
        this.input = input;
        this.output = output;
        this.behavior = behavior;
        this.goalState = goalState;
        this.loggableGoal = new LoggableFunction<>(
            table -> this.goalState.toLog(table), 
            table -> this.goalState.fromLog(table), 
            newGoal -> this.goalState = newGoal
        );
    }

    @Override
    public void periodic() {
        Logger.getInstance().processInputs(GOAL_LOG, loggableGoal);
        Optional.of(input.get())
            .map(peek(inputs -> Logger.getInstance().processInputs(INPUT_LOG, inputs)))
            .map(peek(inputs -> this.currentState = inputs))
            .map(inputs -> behavior.periodic(inputs, goalState))
            .map(peek(outputs -> Logger.getInstance().processInputs(OUTPUT_LOG, outputs)))
            .ifPresent(output::accept);
    }

    public GOAL getGoal() {
        return this.goalState;
    }

    public void setGoal(GOAL goalState) {
        this.goalState = goalState;
    }

    public IN getState() {
        return this.currentState;
    }

    public Behavior<IN, OUT, GOAL, ?> getBehavior() {
        return this.behavior;
    }

    protected <T> UnaryOperator<T> peek(Consumer<T> c) {
        return x -> {
            c.accept(x);
            return x;
        };
    }
}
