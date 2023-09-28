package frc.robot.simulation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.state.ArmInputState;

public class MechanismSim {
    private final Mechanism2d panel;
    private final MechanismRoot2d root;
    private final MechanismLigament2d arm;

    private final ArmSubsystem armSubsystem;

    public MechanismSim(ArmSubsystem arm) {
        this.armSubsystem = arm;

        this.panel = new Mechanism2d(Units.inchesToMeters(100), Units.inchesToMeters(100));
        this.root = panel.getRoot("arm", Units.inchesToMeters(7.35), Units.inchesToMeters(10));
        this.arm = root.append(
            new MechanismLigament2d(
                "Arm",
                Units.inchesToMeters(24.719), 
                0,
                6,
                new Color8Bit(Color.kYellow)
            )
        );

        Logger.getInstance().recordOutput("Robot Simulation", panel);
    }

    public void periodic() {
        ArmInputState currentState = armSubsystem.getState();
        if (currentState != null) {
            this.arm.setAngle(currentState.currentAngleDegrees());

            Logger.getInstance().recordOutput("Robot Simulation", panel);
        }
    }
}
