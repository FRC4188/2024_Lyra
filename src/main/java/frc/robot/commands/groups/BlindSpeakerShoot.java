package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

public class BlindSpeakerShoot extends ParallelCommandGroup {

    /* */
    public BlindSpeakerShoot() {
        addCommands(
            new ParallelDeadlineGroup(
                Commands.waitUntil(() -> 
                    Shooter.getInstance().atMPS(13.0, 1.0) && 
                    Shoulder.getInstance().atGoal(Rotation2d.fromDegrees(32.5), 2.0)).andThen(
                new FeedIntoShooter(12.0).andThen(Commands.waitSeconds(0.25))),
                new SetShooterMPS(() -> 13.0),
                new SetShoulderAngle(() -> 32.5))
        );
    }
}