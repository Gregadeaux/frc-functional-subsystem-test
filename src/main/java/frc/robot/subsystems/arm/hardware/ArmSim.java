package frc.robot.subsystems.arm.hardware;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.state.ArmInputState;
import frc.robot.subsystems.arm.state.ArmOutputState;

public class ArmSim implements ArmHardwareLayerInterface{

    public ArmSim() {}

    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        DCMotor.getNEO(1),
        50,
        SingleJointedArmSim.estimateMOI(Units.inchesToMeters(24.719), Units.lbsToKilograms(20)),
        Units.inchesToMeters(24.719), 
        -2 * Math.PI, 
        2 * Math.PI,
        true
    );

    @Override
    public ArmInputState getState() {
        sim.update(0.02);
        return new ArmInputState(Units.radiansToDegrees(sim.getAngleRads()), Units.radiansToDegrees(sim.getVelocityRadPerSec()));
    }

    @Override
    public void setState(ArmOutputState outputState) {
        outputState.voltage().ifPresent((volts) -> sim.setInputVoltage(volts));
    }
    
}
