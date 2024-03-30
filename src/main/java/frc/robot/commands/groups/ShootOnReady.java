package frc.robot.commands.groups;

import frc.robot.commands.drivetrain.TrackingDrive;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

import javax.sound.midi.Track;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class ShootOnReady extends ParallelDeadlineGroup {
    private Swerve drive = Swerve.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private Shooter shooter = Shooter.getInstance();

    /** Creates a new ShootOnReady. 
     * 
    */
    public ShootOnReady() {
        super(
            Commands.waitUntil(() -> 
                Shooter.getInstance().atMPS() && 
                Shoulder.getInstance().atGoal(Sensors.getInstance().getFormulaShoulderAngle()) && 
                Swerve.getInstance().atGoalAngle(Sensors.getInstance().getFormulaDriveAngle())).andThen(
            new FeedIntoShooter(12.0).withTimeout(0.25)),

            new TrackingDrive(() -> 0.0, () -> 0.0),
            new SetShooterMPS(() -> Sensors.getInstance().getFormulaShooterRPM()),
            new SetShoulderAngle(() -> Sensors.getInstance().getFormulaShoulderAngle().getDegrees())
        );
    }
}
