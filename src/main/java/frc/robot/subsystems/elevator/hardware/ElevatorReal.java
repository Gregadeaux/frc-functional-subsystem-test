package frc.robot.subsystems.elevator.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.elevator.ElevatorState.ElevatorInputState;
import frc.robot.subsystems.elevator.ElevatorState.ElevatorOutputState;

public class ElevatorReal implements ElevatorHardwareInterface {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    double circumference = Units.inchesToMeters(1.756) * Math.PI;
    double gearRatio = 4.0; 

    public ElevatorReal(int motorID){
        motor = new CANSparkMax(motorID, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(true);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(circumference / gearRatio);
    }

    @Override
    public ElevatorInputState getState() {
        return new ElevatorInputState(getVelocity(), getPosition());
    }

    @Override
    public void setState(ElevatorOutputState outputState) {
        outputState.voltage().ifPresent(motor::setVoltage);
    }

    private double getPosition() {
        return encoder.getPosition();
    }

    private double getVelocity() {
        return encoder.getVelocity();
    }
    
}
