// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import CSP_Lib.inputs.CSP_Controller;
import CSP_Lib.inputs.CSP_Controller.Scale;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoConfigs;
import frc.robot.commands.drivetrain.TeleDrive;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.feeder.ReturnToFeeder;
import frc.robot.commands.groups.BlindIntake;
import frc.robot.commands.intake.Exhale;
import frc.robot.commands.intake.Inhale;
import frc.robot.commands.shooter.SetShooterRPM;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

public class RobotContainer {

  // take out later
  // DigitalInput dio0 = new DigitalInput(0);

  // DigitalInput dio2 = new DigitalInput(2);
  // DigitalInput dio3 = new DigitalInput(3);

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

    smartdashboardButtons();

    configureBindings();

    shuffleUpdater.startPeriodic(0.02);

    // Add commands from AutoConfigs to PathPlanner
    NamedCommands.registerCommands(AutoConfigs.EVENTS);
    // Bro what are these names -Aiden

    // Add auto chooser to SmartDashboard
    addChooser();
  }

  private void setDefaultCommands() {
    // drive.setDefaultCommand(
    //   new TeleDrive(
    //     () -> pilot.getLeftY(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.4 : 0.7), 
    //     () -> pilot.getLeftX(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.4 : 0.7), 
    //     () -> pilot.getRightX(Scale.SQUARED) * (pilot.getRightBumperButton().getAsBoolean() ? 0.15 : 0.5))
    // );
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

    pilot
        .getBButton()
        .onTrue(
          new BlindIntake()
        );

    pilot
        .getYButton()
        .onTrue(
          new FeedIntoShooter()
        );

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

    pilot
        .getUpButton()
        .whileTrue(new RunCommand(() -> shoulder.setVoltage(2.5), shoulder))
        .onFalse(new InstantCommand(() -> shoulder.disable()));
        
    pilot
        .getDownButton()
        .whileTrue(new RunCommand(() -> shoulder.setVoltage(-2.5), shoulder))
        .onFalse(new InstantCommand(() -> shoulder.disable()));
  
    pilot
        .getXButton()
        .whileTrue(new RunCommand(() -> shooter.setVoltage(8, 8), shooter))
        .onFalse(new InstantCommand(() -> shooter.disable()));
 
    copilot
        .getBButton()
        .whileTrue(new RunCommand(() -> feeder.set(0.5), feeder))
        .onFalse(new InstantCommand(() -> feeder.disable()));
    
    // buttons for testing SetShooterRPM and SetShoulderAngle :D
    pilot
        .getLeftButton()
        // .whileTrue(new SetShooterRPM(() -> SmartDashboard.getNumber("rpm", 0))); // NOT WORKING FOR SOME REASON
        .whileTrue(new SetShooterRPM(() -> 2000));

    copilot
        .getRightButton()
        .whileTrue(new SetShoulderAngle(() -> SmartDashboard.getNumber("Shoulder Setpoint", 0.0)));

    // Seriously why is it "Inhale" and "Exhale" lmao. I like it though -Aiden
    // Freak you Aiden
  }

  public void updateShuffle() {
    // flywheel.updateDashboard();
    // SmartDashboard.putBoolean("dio 0", dio0.get());
    // SmartDashboard.putBoolean("dio 2", dio2.get());
    // SmartDashboard.putBoolean("dio 3", dio3.get());
  }

  public void smartdashboardButtons() {
    SmartDashboard.putNumber("Shoulder Setpoint", 0.0);
    SmartDashboard.putNumber("RPM Setpoint", 0.0);
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
