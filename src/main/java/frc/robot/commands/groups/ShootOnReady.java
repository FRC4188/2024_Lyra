package frc.robot.commands.groups;

import frc.robot.Constants;
import frc.robot.commands.drivetrain.TrackingDrive;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class ShootOnReady extends ParallelDeadlineGroup {
    /** Creates a new ShootOnReady. 
     * 
    */
    public ShootOnReady(DoubleSupplier xInput, DoubleSupplier yInput) {
        super(
            Commands.waitUntil(() -> 
                Math.abs(Sensors.getInstance().getPigeonRate()) < 90.0 &&
                Shooter.getInstance().atMPS() && 
                Shoulder.getInstance().atGoal(Sensors.getInstance().getFormulaShoulderAngle()) && 
                Swerve.getInstance().atGoalAngle(Sensors.getInstance().getFormulaDriveAngle())).andThen(
            new FeedIntoShooter(12.0).withTimeout(0.25)),

            new TrackingDrive(() -> 0.0, () -> 0.0, () -> Sensors.getInstance().getFormulaDriveAngle().getDegrees()),
            new SetShooterMPS(() -> Sensors.getInstance().getFormulaShooterRPM()),
            new SetShoulderAngle(() -> Sensors.getInstance().getFormulaShoulderAngle().getDegrees())
        );
    }
}
