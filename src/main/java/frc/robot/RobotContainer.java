// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import CSP_Lib.inputs.CSP_Controller;
import CSP_Lib.inputs.CSP_Controller.Scale;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoConfigs;
import frc.robot.commands.drivetrain.HockeyStop;
import frc.robot.commands.drivetrain.TeleDrive;
import frc.robot.commands.drivetrain.XPattern;
import frc.robot.commands.feeder.EjectFeeder;
import frc.robot.commands.feeder.FeedIntoFeeder;
import frc.robot.commands.groups.BlindReverseSpeakerShoot;
import frc.robot.commands.groups.ShooterIntake;
import frc.robot.commands.groups.Amp1;
import frc.robot.commands.groups.Amp2;
import frc.robot.commands.groups.BlindAmpShoot;
import frc.robot.commands.groups.FeedIntake;
import frc.robot.commands.groups.Pass;
import frc.robot.commands.groups.ShootOnReady;
import frc.robot.commands.groups.BlindSpeakerShoot;
import frc.robot.commands.groups.Eject;

import frc.robot.commands.groups.Stow;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;


public class RobotContainer {

  public static CSP_Controller pilot = new CSP_Controller(Constants.controller.PILOT_PORT);
  public static CSP_Controller copilot = new CSP_Controller(Constants.controller.COPILOT_PORT);

  Swerve drive = Swerve.getInstance();
  Intake intake = Intake.getInstance();
  Sensors sensors = Sensors.getInstance();
  Shoulder shoulder = Shoulder.getInstance();
  Shooter shooter = Shooter.getInstance();
  Feeder feeder = Feeder.getInstance();

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
    //   new TeleDrive(
    //     () -> pilot.getLeftY(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0), 
    //     () -> pilot.getLeftX(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0), 
    //     () -> pilot.getRightX(Scale.SQUARED) * (pilot.getRightBumperButton().getAsBoolean() ? 0.1 : 1.0))
    // );
    new XPattern());
  }

  private void configureBindings() {

    //Add these in for sysid tests
    // pilot.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // pilot.b().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // pilot.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // pilot.y().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    Trigger isShooting = pilot.leftTrigger();
    Trigger drivingInput = new Trigger(() -> (pilot.getCorrectedLeft().getNorm() != 0.0 || pilot.getCorrectedRight().getX() != 0.0));


    drivingInput
    .onTrue(    
      new TeleDrive(
        () -> pilot.getCorrectedLeft().getX() * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0), 
        () -> pilot.getCorrectedLeft().getY() * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0), 
        () -> pilot.getRightX(Scale.SQUARED) * (pilot.getRightBumperButton().getAsBoolean() ? 0.1 : 1.0)))
    .onFalse(new HockeyStop().withTimeout(0.5));

    
    // reset pigeon
    pilot
        .getStartButton()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.resetOdometry(
                    new Pose2d(drive.getPose2d().getTranslation(), 
                    Rotation2d.fromDegrees(sensors.getAllianceColor() == DriverStation.Alliance.Red ? 180 : 0)));
                  drive.rotPID.setSetpoint(180.0);
                }, sensors));

    pilot
        .getAButton()
        .onTrue(
          new BlindAmpShoot()
        );

    pilot
        .getYButton()
        .onTrue(
          new ShooterIntake()
        );

    pilot
        .getXButton()
        .whileTrue(
          new ConditionalCommand(
            new BlindSpeakerShoot(), 
            new BlindReverseSpeakerShoot(), 
            () -> (drive.getColorNormRotation().getCos() > 0.0))
        ).onFalse(new Stow());
        
    pilot.getBButton()
        .whileTrue(
          new Pass().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        ).onFalse(new Stow());

    pilot
        .getRightTButton()
        .onTrue(
          new FeedIntake()
        );
    
    pilot.getLeftTButton()
        .whileTrue(
          new ShootOnReady().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        ).onFalse(new Stow());

    pilot 
        .getLeftBumperButton()
        .whileTrue(
          new Eject()
        );

    copilot
        .getAButton()
        .onTrue(
          new Amp1()
        );

    
    copilot
        .getYButton()
        .onTrue(
          new Amp2()
        );


    copilot
        .getUpButton()
        .onTrue(
          new FeedIntoFeeder(1.8)
        );
        
    copilot
        .getDownButton()
        .whileTrue(
          new EjectFeeder()
        );

    copilot
        .getStartButton()
        .onTrue(
          new Stow()
        );

    // Seriously why is it "Inhale" and "Exhale" lmao. I like it though -Aiden
    // Freak you Aiden
  }

  public void updateShuffle() {
    // SmartDashboard.putBoolean("dio0", dio0.get());
    // SmartDashboard.putBoolean("dio1", dio1.get());
    // SmartDashboard.putBoolean("dio4", dio4.get());
    // SmartDashboard.putBoolean("dio5", dio5.get()); 
    
  }

  public void smartdashboardButtons() {
    // SmartDashboard.putNumber("Shoulder Point", 0.0);
    // SmartDashboard.putNumber("MPS Point", 0.0);
  }

  public void addChooser() {
    autoChooser.setDefaultOption("Do Nothing", new SequentialCommandGroup());
  // VISIONLESS AUTONOMOUS PATHS
    // autoChooser.addOption("Blind Four Mid", new PathPlannerAuto("Red Four Mid"));
    // autoChooser.addOption("IHOT Auto", new PathPlannerAuto("IHOT Auto"));
    // autoChooser.addOption("Shoot and Leave", new PathPlannerAuto("Shoot and Leave"));
    // autoChooser.addOption("Red Source One", new RedSourceNoteOne());
    // autoChooser.addOption("Blue Source One", new BlueSourceNoteOne());
    // autoChooser.addOption("Make Them Cry", new PathPlannerAuto("Make Them Cry"));
    // autoChooser.addOption("Red Walton Auto", new PathPlannerAuto("Walton Auto").andThen(new RedSourceNoteOne()));
    // autoChooser.addOption("Blue Walton Auto", new PathPlannerAuto("Walton Auto").andThen(new BlueSourceNoteOne()));

    autoChooser.addOption("5 piece", new PathPlannerAuto("5 piece")); 
    autoChooser.addOption("4.5 piece", new PathPlannerAuto("4.5 piece")); 
    autoChooser.addOption("Source 3 Piece", new PathPlannerAuto("Source 3 Piece"));
    autoChooser.addOption("Source 4 Piece", new PathPlannerAuto("Source 4 Piece"));


    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
