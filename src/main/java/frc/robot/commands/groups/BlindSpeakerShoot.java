package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
                    new SetShooterMPS(() -> 50.0),
                    new SetShoulderAngle(() -> 20.0)
                ).until(() -> shoulder.atGoal(20.0) && shooter.atRPM(50.0)),
                new FeedIntoShooter()
            )
        );
    }
}
