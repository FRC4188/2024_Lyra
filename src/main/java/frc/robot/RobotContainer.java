// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import CSP_Lib.inputs.CSP_Controller;
import CSP_Lib.inputs.CSP_Controller.Scale;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoConfigs;
import frc.robot.commands.drivetrain.TeleDrive;
import frc.robot.commands.intake.Exhale;
import frc.robot.commands.intake.Inhale;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

public class RobotContainer {

  private CSP_Controller pilot = new CSP_Controller(Constants.controller.PILOT_PORT);
  private CSP_Controller copilot = new CSP_Controller(Constants.controller.COPILOT_PORT);

  Swerve drive = Swerve.getInstance();
  Intake intake = Intake.getInstance();
  Sensors sensors = Sensors.getInstance();
  Shoulder shoulder = Shoulder.getInstance();
  Shooter shooter = Shooter.getInstance();
  Feeder feeder = Feeder.getInstance();
  // Flywheel flywheel = Flywheel.getInstance();

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private Notifier shuffleUpdater = new Notifier(() -> updateShuffle());

  public RobotContainer() {
    // Set the default commands
    setDefaultCommands();

    configureBindings();

    smartdashboardButtons();

    shuffleUpdater.startPeriodic(0.02);

    // Add commands from AutoConfigs to PathPlanner
    NamedCommands.registerCommands(AutoConfigs.EVENTS);
    // Bro what are these names -Aiden

    // Add auto chooser to SmartDashboard
    addChooser();
  }

  private void setDefaultCommands() {
    drive.setDefaultCommand(
      new TeleDrive(
        () -> pilot.getLeftY(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.4 : 0.7), 
        () -> pilot.getLeftX(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.4 : 0.7), 
        () -> pilot.getRightX(Scale.SQUARED) * (pilot.getRightBumperButton().getAsBoolean() ? 0.15 : 0.5))
    );
  }

  private void configureBindings() {
    
    //Add these in for sysid tests
    // pilot.a().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // pilot.b().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // pilot.x().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // pilot.y().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    //BRING THIS BACK ONCE WE CAN TRACK AND DRIVE
    // pilot 
    //     .getStartButton()
    //     .whileFalse(
    //       new TeleDrive(
    //         () -> pilot.getLeftY(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.4 : 0.7), 
    //         () -> pilot.getLeftX(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.4 : 0.7), 
    //         () -> pilot.getRightX(Scale.SQUARED) * (pilot.getRightBumperButton().getAsBoolean() ? 0.15 : 0.5))
    //     ).whileTrue(
    //       new ShootOnReady(
    //         () -> pilot.getLeftY(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.4 : 0.7), 
    //         () -> pilot.getLeftX(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.4 : 0.7),
    //         DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Constants.field.BLUE_SPEAKER_LOCATION : Constants.field.RED_SPEAKER_LOCATION
    //       )
    //     );
    
    //     new FeedIntake().andThen(new Stow().until(() -> sensors.areHappy()))


    //reset pigeon
    pilot
        .getAButton()
        .onTrue(
            new InstantCommand(
                () -> {
                  sensors.resetPigeon();
                  drive.rotPID.setSetpoint(0.0);
                }, sensors));

    //intake
    copilot
        .getRightTButton()
        .whileTrue(new Inhale())
        .whileFalse(new InstantCommand(() -> intake.disable(), intake));

    //outake
    copilot
        .getLeftTButton()
        .whileTrue(new Exhale())
        .onFalse(new InstantCommand(() -> intake.disable(), intake));

    copilot
        .getUpButton()
        .whileTrue(new RunCommand(() -> shoulder.setVoltage(0.2), shoulder))
        .onFalse(new InstantCommand(() -> shoulder.disable()));
        
    copilot
        .getDownButton()
        .whileTrue(new RunCommand(() -> shoulder.setVoltage(-0.2), shoulder))
        .onFalse(new InstantCommand(() -> shoulder.disable()));
  
    copilot
        .getAButton()
        .whileTrue(new RunCommand(() -> shooter.set(0.5, 0.5), shooter))
        .onFalse(new InstantCommand(() -> shooter.disable()));

    copilot
        .getBButton()
        .whileTrue(new RunCommand(() -> feeder.set(0.5), feeder))
        .onFalse(new InstantCommand(() -> feeder.disable()));

    // Seriously why is it "Inhale" and "Exhale" lmao. I like it though -Aiden
    // Freak you Aiden
  }

  public void updateShuffle() {
    // flywheel.updateDashboard();
  }

  public void smartdashboardButtons() {
    SmartDashboard.putNumber("Shoulder Setpoint", 0.0);
    SmartDashboard.putData("Set Shoulder Angle", new SetShoulderAngle(() -> SmartDashboard.getNumber("Shoulder Setpoint", 0.0)));

    SmartDashboard.putNumber("Shooter Voltage Setpoint", 0.0);
    SmartDashboard.putData("Set Shooter Velocity", new SetShooterMPS(() -> SmartDashboard.getNumber("Shooter Velocity Setpoint", 0.0)));
  }

  public void addChooser() {
    autoChooser = AutoBuilder.buildAutoChooser();
    
    autoChooser.setDefaultOption("Do Nothing", new SequentialCommandGroup());
    autoChooser.addOption("Three Deep Breaths", new PathPlannerAuto("Three Deep Breaths"));
    //autoChooser.addOption("First Note", new PathPlannerAuto("First Note"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
