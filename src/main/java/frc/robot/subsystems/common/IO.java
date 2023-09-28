package frc.robot.subsystems.common;

public interface IO<T, F> {
    public T getState();
    public void setState(F outputState);
}
