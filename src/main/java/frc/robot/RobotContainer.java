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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoConfigs;
import frc.robot.commands.climber.LowerClimber;
import frc.robot.commands.climber.RaiseClimber;
import frc.robot.commands.drivetrain.TeleDrive;
import frc.robot.commands.feeder.EjectFeeder;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.groups.ReverseBlindSpeakerPrep;
import frc.robot.commands.groups.ReverseBlindAmpShoot;
import frc.robot.commands.groups.BlindAmpShoot;
import frc.robot.commands.groups.BlindIntake;
import frc.robot.commands.groups.BlindSpeakerPrep;
import frc.robot.commands.groups.Eject;
import frc.robot.commands.groups.StillSpeakerPrep;
import frc.robot.commands.groups.Stow;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class RobotContainer {

  DigitalInput dio1 = new DigitalInput(1);
  DigitalInput dio4 = new DigitalInput(4);

  private CSP_Controller pilot = new CSP_Controller(Constants.controller.PILOT_PORT);
  private CSP_Controller copilot = new CSP_Controller(Constants.controller.COPILOT_PORT);

  Swerve drive = Swerve.getInstance();
  Intake intake = Intake.getInstance();
  Sensors sensors = Sensors.getInstance();
  Shoulder shoulder = Shoulder.getInstance();
  Shooter shooter = Shooter.getInstance();
  Feeder feeder = Feeder.getInstance();
  Climber climber = Climber.getInstance();

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
    drive.setDefaultCommand(
      new TeleDrive(
        () -> pilot.getLeftY(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.5 : 1.0), 
        () -> pilot.getLeftX(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.5 : 1.0), 
        () -> pilot.getRightX(Scale.SQUARED) * (pilot.getRightBumperButton().getAsBoolean() ? 0.1 : 1.0))
    );
  }

  private void configureBindings() {
    
    //Add these in for sysid tests
    // pilot.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // pilot.b().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // pilot.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // pilot.y().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    //BRING THIS BACK ONCE WE CAN TRACK AND DRIVE TODO: ACTUALLY, DONT 
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


    // reset pigeon
    pilot
        .getAButton()
        .onTrue(
            new InstantCommand(
                () -> {
                  sensors.resetPigeon();
                  drive.rotPID.setSetpoint(0.0);
                }, sensors));

    pilot
        .getRightTButton()
        .onTrue(
          new BlindIntake()
        );

    //outtake intake
    pilot 
        .getLeftBumperButton()
        .whileTrue(
          new Eject()
        );

    pilot
        .getLeftTButton()
        .whileTrue(
          new FeedIntoShooter()
        );

    //outtake feeder
    pilot
        .getDownButton()
        .whileTrue(
          new EjectFeeder()
        );

    copilot
        .getAButton()
        .onTrue(
          new BlindAmpShoot()
        );

    //shooter on battery side w speaker angle
    copilot
        .getYButton()
        .onTrue(
          new BlindSpeakerPrep()
        );

    //shooter on intake side w speaker angle
    copilot
        .getBButton()
        .onTrue(
          new ReverseBlindSpeakerPrep()
        );
    
        copilot
        .getXButton()
        .onTrue(
          new ReverseBlindAmpShoot()
        );

    //default shooter pos + stop intake n feeder
    copilot
        .getStartButton()
        .onTrue(
          new Stow()
        );

    copilot
        .getRightTButton()
        .onTrue(
          new SetShoulderAngle(() -> SmartDashboard.getNumber("Shoulder Point", 0.0))
          .alongWith(
            new SetShooterMPS(() -> SmartDashboard.getNumber("MPS Point", 0.0))
          )
        );

    // copilot
    //     .getUpButton()
    //     .onTrue(
    //       new RaiseClimber()
    //     );

    // copilot
    //     .getDownButton()
    //     .whileTrue(
    //       new LowerClimber()
    //     );
  
    // Seriously why is it "Inhale" and "Exhale" lmao. I like it though -Aiden
    // Freak you Aiden
  }

  public void updateShuffle() {
    // SmartDashboard.putBoolean("dio0", dio0.get());
    SmartDashboard.putBoolean("dio1", dio1.get());
    SmartDashboard.putBoolean("dio4", dio4.get());
    // SmartDashboard.putBoolean("dio5", dio5.get()); 

    
  }

  public void smartdashboardButtons() {
    SmartDashboard.putNumber("Shoulder Point", 0.0);
    SmartDashboard.putNumber("MPS Point", 0.0);
  }

  public void addChooser() {
    autoChooser = AutoBuilder.buildAutoChooser();
    
    autoChooser.setDefaultOption("Do Nothing", new SequentialCommandGroup());
    autoChooser.addOption("Three Deep Breaths", new PathPlannerAuto("Three Deep Breaths"));
    autoChooser.addOption("Shoot and Leave", new PathPlannerAuto("Shoot and Leave"));
    //autoChooser.addOption("First Note", new PathPlannerAuto("First Note"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
