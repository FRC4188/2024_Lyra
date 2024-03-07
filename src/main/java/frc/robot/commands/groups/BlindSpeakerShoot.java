package frc.robot.commands.groups;

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
    Shoulder shoulder = Shoulder.getInstance();
    Shooter shooter = Shooter.getInstance();

    public BlindSpeakerShoot() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetShooterMPS(() -> 30.0),
                    new SetShoulderAngle(() -> 60.0)
                ).until(() -> shoulder.atGoal(60.0) && shooter.atRPM(30.0)), 
                new ParallelDeadlineGroup(
                    new FeedIntoShooter().andThen(Commands.waitSeconds(0.25)), 
                    new SetShooterMPS(() -> 30.0),
                    new SetShoulderAngle(() -> 60.0))
            )
        );
    }
}
