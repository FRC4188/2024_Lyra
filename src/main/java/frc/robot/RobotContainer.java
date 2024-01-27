// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import CSP_Lib.inputs.CSP_Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.shooter.Flywheel;

public class RobotContainer {
  CSP_Controller pilot = new CSP_Controller(0);
  Flywheel flywheel = Flywheel.getInstance();
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    flywheel.setDefaultCommand(new RunCommand(() -> flywheel.set(1.0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
// im raymond and im trash