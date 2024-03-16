// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import CSP_Lib.inputs.CSP_Controller;
import CSP_Lib.utils.Binding;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooting.BlindShoot;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final CSP_Controller pilot, copilot;

  public RobotContainer(CSP_Controller pilot, CSP_Controller copilot) {
    this.pilot = pilot;
    this.copilot = copilot;

    setDefaultCommands();

    configureBindings();

    // Add auto chooser to SmartDashboard
    addChooser();
  }

  private void setDefaultCommands() {
  }

  private void configureBindings() {

    // The below configuration is pretty useless it just shows how
    // to use Bindings to constrain a button binding to a state.
    
    new Binding(
      pilot.rightTrigger(), // Trigger
      new BlindShoot(BlindShots.SPEAKER_DIRECT), // On True
      null // On False
    ).addState(RobotControlState.SHOOT); // Add valid state

    new Binding(
      pilot.rightTrigger(), // Trigger
      new BlindShoot(BlindShots.SPEAKER_DEFENDED), // On True
      null // On False
    ).addState(RobotControlState.DRIVE) // Add valid state
    .addState(RobotControlState.BLIND_SHOOT); // Add another state
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
