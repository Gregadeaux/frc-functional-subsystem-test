package frc.robot.subsystems.common;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

public abstract class Behavior<IN, OUT, GOAL, STATE>{
    private STATE state;
    private Consumer<STATE> stateConsumer;
    private List<Plug<OUT>> plugs = new LinkedList<>();

    public abstract OUT periodic(final IN inputs, final GOAL goal, final STATE state);
    public final OUT periodic(final IN inputs, final GOAL goal) {
        OUT output = this.periodic(inputs, goal, state);
        for(Plug<OUT> plug : plugs) {
            output = plug.effect(output);
        }
        return output;
    }

    protected final void setState(final STATE state) {
        this.state = state;
        if(this.stateConsumer != null) stateConsumer.accept(this.state);
    }

    public final void setStateConsumer(Consumer<STATE> newConsumer) {
        if(this.stateConsumer != null) {
            this.stateConsumer = null;
        }
        this.stateConsumer = newConsumer;
    }

    public final Behavior<IN, OUT, GOAL, STATE> plug(Plug<OUT> plug) {
        this.plugs.add(plug);
        return this;
    }
}
