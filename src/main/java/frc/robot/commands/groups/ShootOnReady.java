package frc.robot.commands.groups;

import frc.robot.commands.drivetrain.TrackingDrive;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.shooter.SetShooterRPM;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootOnReady extends ParallelCommandGroup {
    private Swerve drive = Swerve.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private Sensors sensors = Sensors.getInstance();

    /** Creates a new ShootOnReady. 
     * 
    */
    public ShootOnReady(DoubleSupplier xInput, DoubleSupplier yInput) {
        addCommands(
                new ParallelDeadlineGroup(
                    Commands.waitUntil(() -> 
                        shooter.atRPM(sensors.getMovingShooterRPM()) && 
                        shoulder.atGoal(sensors.getMovingShoulderAngle().getDegrees()) && 
                        drive.atGoalAngle(sensors.getMovingDriveAngle().getDegrees())).andThen(
                    new FeedIntoShooter().andThen(Commands.waitSeconds(0.25))),
                    new SetShooterRPM(() -> sensors.getMovingShooterRPM()),
                    new SetShoulderAngle(() -> sensors.getMovingShoulderAngle().getDegrees()),
                    new TrackingDrive(xInput, yInput)
                )
        );
    }
}
