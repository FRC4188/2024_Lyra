// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import CSP_Lib.inputs.CSP_Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.drivetrain.TeleDrive;
import frc.robot.subsystems.drivetrain.Swerve;

public class RobotContainer {

  Swerve drive = Swerve.getInstance();

  private CSP_Controller pilot = new CSP_Controller(Constants.controller.PILOT_PORT);
  private CSP_Controller copilot = new CSP_Controller(Constants.controller.COPILOT_PORT);

  public RobotContainer() {
    // Set the default commands
    setDefaultCommands();

    configureBindings();
  }

  private void setDefaultCommands() {
    drive.setDefaultCommand(
      new TeleDrive(() -> pilot.getLeftX(), () -> pilot.getLeftY(), () -> pilot.getRightX())
    );
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
