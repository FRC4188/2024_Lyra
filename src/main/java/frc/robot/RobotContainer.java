// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import CSP_Lib.inputs.CSP_Controller;
import CSP_Lib.inputs.CSP_Controller.Scale;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoConfigs;
import frc.robot.commands.drivetrain.HockeyStop;
import frc.robot.commands.drivetrain.TeleDrive;
import frc.robot.commands.drivetrain.XPattern;
import frc.robot.commands.feeder.EjectFeeder;
import frc.robot.commands.feeder.FeedIntoFeeder;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.groups.BlindReverseSpeakerShoot;
import frc.robot.commands.groups.ShooterIntake;
import frc.robot.commands.groups.BlindAmpShoot;
import frc.robot.commands.groups.BlindPass;
import frc.robot.commands.groups.BlindReversePass;
import frc.robot.commands.groups.FeedIntake;
import frc.robot.commands.groups.PassOnReady;
import frc.robot.commands.groups.ShootOnReady;
import frc.robot.commands.groups.BlindSpeakerShoot;
import frc.robot.commands.groups.Eject;

import frc.robot.commands.groups.Stow;
import frc.robot.commands.groups.autos.RobotTest;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.LED;
import frc.robot.subsystems.sensors.LEDState;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;


public class RobotContainer {

  public static CSP_Controller pilot = new CSP_Controller(Constants.controller.PILOT_PORT);
  public static CSP_Controller copilot = new CSP_Controller(Constants.controller.COPILOT_PORT);

  // Swerve drive = Swerve.getInstance();
  Intake intake = Intake.getInstance();
  Sensors sensors = Sensors.getInstance();
  Shoulder shoulder = Shoulder.getInstance();
  Shooter shooter = Shooter.getInstance();
  Feeder feeder = Feeder.getInstance();
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  LED led = LED.getInstance();
  private Notifier shuffleUpdater = new Notifier(() -> updateShuffle());

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

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
    // // //   new TeleDrive(
    // // //     () -> pilot.getLeftY(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0), 
    // // //     () -> pilot.getLeftX(Scale.LINEAR) * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0), 
    // // //     () -> pilot.getRightX(Scale.SQUARED) * (pilot.getRightBumperButton().getAsBoolean() ? 0.1 : 1.0))
    // // // );

    // );

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.brake()
    );

    led.setDefaultCommand(
      new ConditionalCommand(
      new InstantCommand(() -> led.turnOn(LEDState.Orange), led),
      new InstantCommand(() -> led.turnOn(LEDState.BlueGreen), led),
      () -> feeder.isBroken())
    );

    //led.setDefaultCommand(new InstantCommand(() -> led.turnOn(LEDState.Dashboard), led));

  }

  private void configureBindings() {

    SmartDashboard.putNumber("Angle", 0.0);
    SmartDashboard.putNumber("Velocity", 0.0);
    SmartDashboard.putData("Set Shooter MPS", new SetShooterMPS(() -> (SmartDashboard.getNumber("Velocity", 0))));
    SmartDashboard.putData("Set Shoulder Angle", new SetShoulderAngle(() -> (SmartDashboard.getNumber("Angle", 0))));
    //Add these in for sysid tests
    // pilot.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // pilot.b().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // pilot.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // pilot.y().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putNumber("R", 0);
        SmartDashboard.putNumber("B", 0);
    SmartDashboard.putNumber("G", 0);



    // Trigger isShooting = pilot.leftTrigger();
    Trigger drivingInput = new Trigger(() -> (pilot.getCorrectedLeft().getNorm() != 0.0 || pilot.getCorrectedRight().getX() != 0.0));

    // pilot.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-pilot.getLeftY(), -pilot.getLeftX()))));

    // reset the field-centric heading on left bumper press
    

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    
    // drivingInput
    // .onTrue(   drivetrain.applyRequest(() -> drive.withVelocityX(pilot.getCorrectedLeft().getX() * (pilot.getRightBumperButton().getAsBoolean() ? 0.25 : 1.0)) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(pilot.getCorrectedLeft().getY() * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0)) // Drive left with negative X (left) 
    //         .withRotationalRate(pilot.getRightX(Scale.SQUARED) * (pilot.getRightBumperButton().getAsBoolean() ? 0.1 : 0.7))) // Drive counterclockwise with negative X (left)
    //     )  
    // .onFalse(new HockeyStop().withTimeout(0.5));

    drivingInput.onTrue(drivetrain.drive(-pilot.getCorrectedLeft().getX() * 1.0, // Drive forward with
                                                                                           // negative Y (forward)
            -pilot.getCorrectedLeft().getY() * 1.0, // Drive left with negative X (left)
            pilot.getRightX(Scale.SQUARED) * 0.7 // Drive counterclockwise with negative X (left)
        ));

    
    // reset pigeon
    // pilot
    //     .getStartButton()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               drive.resetOdometry(
    //                 new Pose2d(drive.getPose2d().getTranslation(), 
    //                 Rotation2d.fromDegrees(sensors.getAllianceColor() == DriverStation.Alliance.Red ? 180 : 0)));
    //               drive.rotPID.setSetpoint(180.0);
    //             }, sensors));

    pilot.getStartButton().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

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

    // pilot
    //     .getXButton()
    //     .whileTrue(
    //       new ConditionalCommand(
    //         new BlindSpeakerShoot(), 
    //         new BlindReverseSpeakerShoot(), 
    //         () -> (drive.getColorNormRotation().getCos() > 0.0))
    //     ).onFalse(new Stow());
        
    // pilot
    //     .getUpButton()
    //     .whileTrue(
    //       new ConditionalCommand(
    //         new BlindPass(), 
    //         new BlindReversePass(), 
    //         () -> (drive.getColorNormRotation().getCos() > 0.0))
    //     ).onFalse(new Stow());

    pilot
        .getBButton()
        .whileTrue(
          new PassOnReady(() -> 0.0, () -> 0.0).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        ).onFalse(new Stow());

    pilot
        .getRightTButton()
        .onTrue(
          new FeedIntake()
        );
    
    pilot.getLeftTButton()
        .whileTrue(
          new ShootOnReady(() -> 0.0, () -> 0.0).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        ).onFalse(new Stow());

    // pilot.getLeftTButton()
    //     .whileTrue(
    //       new ShootOnReady(
    //     () -> pilot.getCorrectedLeft().getX() * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0), 
    //     () -> pilot.getCorrectedLeft().getY() * (pilot.getRightBumperButton().getAsBoolean() ? 0.125 : 1.0)
    //       ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    //     ).onFalse(new Stow());

    pilot 
        .getLeftBumperButton()
        .whileTrue(
          new Eject()
        );

    copilot
        .getUpButton()
        .onTrue(
          new FeedIntoFeeder(1.8)
        );

    copilot
        .getRightTButton()
        .whileTrue(
          new FeedIntoShooter(12.0)
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

    
  }

  public void smartdashboardButtons() {

  }

  public void addChooser() {
    autoChooser.setDefaultOption("Do Nothing", new SequentialCommandGroup());
  // VISIONLESS AUTONOMOUS PATHS
    // autoChooser.addOption("Blind Four Mid", new PathPlannerAuto("Red Four Mid"));
    // autoChooser.addOption("IHOT Auto", new PathPlannerAuto("IHOT Auto"));
    autoChooser.addOption("Shoot and Leave RLBR", new PathPlannerAuto("Shoot and Leave"));
    autoChooser.addOption("Shoot and Leave RMBM", new PathPlannerAuto("Shoot and Leave Mid"));
    autoChooser.addOption("Shoot and Leave RRBL", new PathPlannerAuto("Shoot and Leave Right"));
    autoChooser.addOption("Fall wall S&L", new PathPlannerAuto("wall leave"));
    autoChooser.addOption("Blind and Stay", new SequentialCommandGroup(new BlindReverseSpeakerShoot().withTimeout(3.5), new Stow()));
    //autoChooser.addOption("Leave then Shoot Wall", new PathPlannerAuto("wall leave"));
    
    // autoChooser.addOption("Red Source One", new RedSourceNoteOne());
    // autoChooser.addOption("Blue Source One", new BlueSourceNoteOne());
    // autoChooser.addOption("Make Them Cry", new PathPlannerAuto("Make Them Cry"));
    // autoChooser.addOption("Red Walton Auto", new PathPlannerAuto("Walton Auto").andThen(new RedSourceNoteOne()));
    // autoChooser.addOption("Blue Walton Auto", new PathPlannerAuto("Walton Auto").andThen(new BlueSourceNoteOne()));

    // autoChooser.addOption("5 piece", new PathPlannerAuto("5 piece")); 
    // autoChooser.addOption("Middle Wing 4.5 piece", new PathPlannerAuto("Middle Wing 4.5 piece")); 
    // autoChooser.addOption("Amp Wing 4.5 piece", new PathPlannerAuto("Amp Wing 4.5 piece")); 
    // autoChooser.addOption("Source Wing 4 piece", new PathPlannerAuto("Source Wing 4 piece")); 

    // autoChooser.addOption("Source 3 Piece", new PathPlannerAuto("Source 3 Piece"));
    // autoChooser.addOption("Source 4 Piece", new PathPlannerAuto("Source 4 Piece"));

    // autoChooser.addOption("Techno", new PathPlannerAuto("Techno"));

    // autoChooser.addOption("test follow", new RobotTest());
    // autoChooser.addOption("Testing plase", new PathPlannerAuto("TestAuto"));


    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
