package frc.robot.subsystems.common;

public interface Behavior<IN, OUT, GOAL> {
    public OUT periodic(IN inputs, GOAL goal);
}
