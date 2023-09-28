package frc.robot.subsystems.arm.hardware;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.Preferences;
import frc.robot.subsystems.arm.state.ArmInputState;
import frc.robot.subsystems.arm.state.ArmOutputState;

public class ArmReal implements ArmHardwareLayerInterface {

    private static final String PREFERENCE_NAME = "ArmOffsetDegrees";
    private static double armOffsetDegrees = Preferences.getDouble(PREFERENCE_NAME, 0);
    private static double offset = 36.0;
    private static final int motorID = 0;

    private CANSparkMax motor;
    private AbsoluteEncoder encoder;

    public ArmReal() {
        motor = new CANSparkMax(motorID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        motor.setInverted(true);

        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(360);
        encoder.setVelocityConversionFactor(60);
        encoder.setInverted(true);
        encoder.setZeroOffset((armOffsetDegrees + offset) % 360);
    }

    @Override
    public ArmInputState getState() {
        return new ArmInputState(getCurrentAngleDegrees(), getVelocityDegreesPerSecond());
    }

    private double getCurrentAngleDegrees() {
        return this.encoder.getPosition();
    }

    private double getVelocityDegreesPerSecond() {
        return this.encoder.getVelocity();
    }

    @Override
    public void setState(ArmOutputState outputState) {
        outputState.adjustOffsetDegrees().ifPresent(this::adjustOffsetDegrees);
        outputState.voltage().ifPresent(this::setVoltage);
    }

    private void adjustOffsetDegrees(double offsetDegrees) {
        armOffsetDegrees += offsetDegrees;
        Preferences.setDouble(PREFERENCE_NAME, armOffsetDegrees);
        encoder.setZeroOffset((armOffsetDegrees + offset) % 360);
    }
    
    private void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
