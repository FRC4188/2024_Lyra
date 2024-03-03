package frc.robot.commands.groups;

import frc.robot.commands.drivetrain.TrackingDrive;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootOnReady extends ParallelCommandGroup {
    private Swerve drive = Swerve.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private Sensors sensors = Sensors.getInstance();

    /** Creates a new ShootOnReady. 
     * 
    */
    public ShootOnReady(DoubleSupplier xInput, DoubleSupplier yInput, Translation3d goal) {
        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetShooterMPS(() -> sensors.getMovingShooterRPM(goal)),
                    new SetShoulderAngle(() -> sensors.getMovingShoulderAngle(goal).getDegrees()),
                    new TrackingDrive(xInput, yInput, goal)
                ).until(
                    () -> 
                        shooter.atRPM(sensors.getMovingShooterRPM(goal)) && 
                        shoulder.atGoal(sensors.getMovingShoulderAngle(goal).getDegrees()) && 
                        drive.atGoalAngle(sensors.getMovingDriveAngle(goal).getDegrees())),
                new FeedIntoShooter()
            )
        );
    }
}
