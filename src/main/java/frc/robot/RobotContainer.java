// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import CSP_Lib.inputs.CSP_Controller;
import CSP_Lib.inputs.CSP_Controller.Scale;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.drivetrain.TeleDrive;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.intake.Intake;

public class RobotContainer {

  Swerve drive = Swerve.getInstance();
  Intake intake = Intake.getInstance();

  private CSP_Controller pilot = new CSP_Controller(Constants.controller.PILOT_PORT);
  private CSP_Controller copilot = new CSP_Controller(Constants.controller.COPILOT_PORT);

  public RobotContainer() {
    // Set the default commands
    setDefaultCommands();

    configureBindings();
  }

  private void setDefaultCommands() {
    drive.setDefaultCommand(
      pilot.rightBumper().getAsBoolean()
      ? new TeleDrive(() -> pilot.getLeftX() * 0.5, () -> pilot.getLeftY() * 0.5, () -> pilot.getRightX() * 0.1) 
      : new TeleDrive(() -> pilot.getLeftX(), () -> pilot.getLeftY(), () -> pilot.getRightX()) //slow
    );
  }

  private void configureBindings() {
    pilot
        .getLeftTButton()
        .whileTrue(new RunCommand(() -> intake.intake(), intake))
        .onFalse(new InstantCommand(() -> intake.disable(), intake));
    pilot
        .getRightTButton()
        .whileTrue(new RunCommand(() -> intake.outtake(), intake))
        .onFalse(new InstantCommand(() -> intake.disable(), intake));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
