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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PassOnReady extends ParallelDeadlineGroup {
    /** Creates a new ShootOnReady. 
     * 
    */
    public PassOnReady(DoubleSupplier xInput, DoubleSupplier yInput) {
        super(
            new ParallelRaceGroup(
                new WaitCommand(1.5), 
                Commands.waitUntil(() -> 
                    Math.abs(Sensors.getInstance().getPigeonRate()) < 90.0 &&
                    Shooter.getInstance().atMPS(2.0) && 
                    Shoulder.getInstance().atGoal(Rotation2d.fromDegrees(-40.0).times(-Math.signum(Swerve.getInstance().getColorNormRotation().getCos())), 3.0) && 
                    Swerve.getInstance().atGoalAngle(Sensors.getInstance().getCornerDriveAngle())))
            .andThen(
            new FeedIntoShooter(12.0).withTimeout(0.25)),

            new TrackingDrive(() -> 0.0, () -> 0.0, () -> Sensors.getInstance().getCornerDriveAngle().getDegrees()),
            new SetShooterMPS(() -> Sensors.getInstance().getCornerShooterRPM()),
            new SetShoulderAngle(() -> -40.0 * -Math.signum(Swerve.getInstance().getColorNormRotation().getCos()))
        );
    }
}
