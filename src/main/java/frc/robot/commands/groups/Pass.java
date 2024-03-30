package frc.robot.commands.groups;

import frc.robot.Constants;
import frc.robot.commands.drivetrain.TrackingDrive;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;
import edu.wpi.first.wpilibj.DriverStation;

import javax.sound.midi.Track;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class Pass extends ParallelDeadlineGroup {
    private Swerve drive = Swerve.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private Shooter shooter = Shooter.getInstance();

    /** Creates a new ShootOnReady. 
     * 
    */
    public Pass() {
        super(
            Commands.waitUntil(() -> 
                Shooter.getInstance().atMPS(16.0) && 
                Shoulder.getInstance().atGoal(Rotation2d.fromDegrees(-30.0)) && 
                (Swerve.getInstance().atGoalAngle(Sensors.getInstance().getFormulaDriveAngle(
                    Sensors.getInstance().getAllianceColor() == DriverStation.Alliance.Blue ? 
                        Constants.field.BLUE_AMP_LOCATION.toTranslation2d() :
                        Constants.field.RED_AMP_LOCATION.toTranslation2d())) ||
                Swerve.getInstance().atGoalAngle(Sensors.getInstance().getFormulaDriveAngle(
                    Sensors.getInstance().getAllianceColor() == DriverStation.Alliance.Blue ? 
                        Constants.field.BLUE_AMP_LOCATION.toTranslation2d() :
                        Constants.field.RED_AMP_LOCATION.toTranslation2d()
                ).rotateBy(Rotation2d.fromRotations(0.5))))).andThen(
            new FeedIntoShooter(12.0).withTimeout(0.25)),

            new TrackingDrive(() -> 0.0, () -> 0.0),
            new SetShooterMPS(() -> 16.0),
            new SetShoulderAngle(() -> -30.0)
        );
    }
}
