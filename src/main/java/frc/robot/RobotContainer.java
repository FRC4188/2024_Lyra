// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import CSP_Lib.inputs.CSP_Controller;
import CSP_Lib.inputs.CSP_Controller.Scale;
import CSP_Lib.utils.Binding;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShotConstants.BlindShots;
import frc.robot.commands.drivetrain.TeleDrive;
import frc.robot.commands.groups.BlindShoot;

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

    Trigger drivingInput = new Trigger(() -> MathUtil.applyDeadband(pilot.getLeftX(), 0.075) != 0.0 || MathUtil.applyDeadband(pilot.getLeftY(), 0.075) != 0.0 || MathUtil.applyDeadband(pilot.getRightX(), 0.075) != 0.0);
    drivingInput.onTrue(new TeleDrive(
        () -> pilot.getLeftY(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0), 
        () -> pilot.getLeftX(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0), 
        () -> pilot.getRightX(Scale.SQUARED) * (pilot.getRightBumperButton().getAsBoolean() ? 0.1 : 1.0))
    );
    // .onFalse(new HockeyStop().withTimeout(0.25));


    // The below configuration is pretty useless it just shows how
    // to use Bindings to constrain a button binding to a state.
    
    // new Binding(
    //   pilot.rightTrigger(), // Trigger
    //   new BlindShoot(BlindShots.SPEAKER_DIRECT), // On True
    //   null // On False
    // ).addState(RobotControlState.SHOOT); // Add valid state

    // new Binding(
    //   pilot.rightTrigger(), // Trigger
    //   new BlindShoot(BlindShots.SPEAKER_DEFENDED), // On True
    //   null // On False
    // ).addState(RobotControlState.DRIVE) // Add valid state
    // .addState(RobotControlState.BLIND_SHOOT); // Add another state
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
