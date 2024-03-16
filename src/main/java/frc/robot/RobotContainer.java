// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import CSP_Lib.inputs.CSP_Controller;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final Notifier shuffleUpdater = new Notifier(() -> updateShuffle());
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final CSP_Controller pilot, copilot;

  private RobotControlState state = RobotControlState.STOW;

  public RobotContainer(CSP_Controller pilot, CSP_Controller copilot) {
    this.pilot = pilot;
    this.copilot = copilot;

    setDefaultCommands();

    configureBindings();

    shuffleUpdater.startPeriodic(0.1);

    // Add auto chooser to SmartDashboard
    addChooser();
  }

  private void setDefaultCommands() {
  }

  private void configureBindings() {
  }

  public void updateShuffle() {
  }

  public void addChooser() {
     autoChooser.setDefaultOption("Do Nothing", null);
     SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
