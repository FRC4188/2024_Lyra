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

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class PassOnReady extends ParallelDeadlineGroup {
    /** Creates a new ShootOnReady. 
     * 
    */
    public PassOnReady(DoubleSupplier xInput, DoubleSupplier yInput) {
        super(
            Commands.waitUntil(() -> 
                Shooter.getInstance().atMPS(2.0) && 
                Shoulder.getInstance().atGoal(Rotation2d.fromDegrees(-40.0), 3.0) && 
                Swerve.getInstance().atGoalAngle(Sensors.getInstance().getFormulaDriveAngle(
                    DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Constants.field.BLUE_CORNER_LOCATION : Constants.field.RED_CORNER_LOCATION
                ))).andThen(
            new FeedIntoShooter(12.0).withTimeout(0.25)),

            new TrackingDrive(() -> 0.0, () -> 0.0, Constants.field.RED_CORNER_LOCATION),
            new SetShooterMPS(() -> Sensors.getInstance().getFormulaShooterRPM()),
            new SetShoulderAngle(() -> -40.0)
        );
    }
}
