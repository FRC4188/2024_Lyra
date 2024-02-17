// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.management.relation.RoleInfo;

import CSP_Lib.inputs.CSP_Controller;
import CSP_Lib.inputs.CSP_Controller.Scale;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.drivetrain.TeleDrive;
import frc.robot.commands.intake.Exhale;
import frc.robot.commands.intake.Inhale;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.Sensors;

public class RobotContainer {

  Swerve drive = Swerve.getInstance();
  Intake intake = Intake.getInstance();
  Sensors sensors = Sensors.getInstance();

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
      ? new TeleDrive(() -> pilot.getLeftY(Scale.LINEAR) * 0.1, () -> pilot.getLeftX(Scale.LINEAR) * 0.1, () -> pilot.getRightX(Scale.SQUARED) * 0.05) 
      : new TeleDrive(() -> pilot.getLeftY(Scale.LINEAR) * 0.5, () -> pilot.getLeftX(Scale.LINEAR) * 0.5, () -> pilot.getRightX(Scale.SQUARED) * 0.1) //slow
    );
  }

  private void configureBindings() {
    pilot
        .getAButton()
        .onTrue(
            new InstantCommand(
                () -> Sensors.getInstance().resetPigeon(),
                sensors));
    pilot
        .getRightTButton()
        .whileTrue(new Inhale())
        .onFalse(new InstantCommand(() -> intake.disable(), intake));
    pilot
        .getLeftTButton()
        .whileTrue(new Exhale())
        .onFalse(new InstantCommand(() -> intake.disable(), intake));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
