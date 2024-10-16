package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

public class BlindReverseSpeakerShoot extends SequentialCommandGroup {

    public BlindReverseSpeakerShoot() {
        addCommands(        
            new ParallelDeadlineGroup(
                Commands.waitUntil(() -> 
                    Shooter.getInstance().atMPS(1.5) && 
                    Shoulder.getInstance().atGoal(Rotation2d.fromDegrees(-25.0), 3.0)).andThen(
                new FeedIntoShooter(12.0).andThen(Commands.waitSeconds(0.25))),
                new SetShooterMPS(() -> 10.5),
                new SetShoulderAngle(() -> -25.0)
            )
        );
    }
}