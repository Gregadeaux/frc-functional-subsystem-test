// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.simulation.MechanismSim;
import frc.robot.subsystems.arm.ArmDefaultBehavior;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.hardware.ArmHardwareLayerInterface;
import frc.robot.subsystems.arm.hardware.ArmSim;
import frc.robot.subsystems.arm.state.ArmOutputState;
import frc.robot.subsystems.common.ClampPlug;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer;
  private ArmSubsystem arm;
  private MechanismSim sim;

  @Override
  public void robotInit() {
    Logger.getInstance().recordMetadata("ProjectName", "ExampleRobot"); // Set a metadata value

    if (isReal()) {
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
        setUseTiming(false); // Run as fast as possible
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }

    // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.getInstance().start();

    ArmHardwareLayerInterface armIO = new ArmSim();
    ArmDefaultBehavior armBusiness = (ArmDefaultBehavior) new ArmDefaultBehavior().plug(
      new ClampPlug<ArmOutputState>(-1, 1,
        (ArmOutputState out) -> out.voltage(),
        (Optional<Double> clamped, ArmOutputState original) -> new ArmOutputState(clamped.or(original::voltage), original.adjustOffsetDegrees())
      )
    );
    this.arm = new ArmSubsystem(armIO, armBusiness);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() { }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    this.sim = new MechanismSim(this.arm);
  }

  @Override
  public void simulationPeriodic() {
    this.sim.periodic();
  }
}
