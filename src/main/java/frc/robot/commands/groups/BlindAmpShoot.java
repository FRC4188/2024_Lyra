package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

public class BlindAmpShoot extends ParallelCommandGroup {
    Shoulder shoulder = Shoulder.getInstance();
    Shooter shooter = Shooter.getInstance();

    public BlindAmpShoot() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetShoulderAngle(() -> 10.0),
                    new SetShooterMPS(() -> 2.0)
                ).until(() -> shooter.atMPS() && shoulder.atGoal(10.0)),
                new ParallelCommandGroup(
                    Commands.waitSeconds(0.2).andThen(new FeedIntoShooter(12.0)),
                    new SetShooterMPS(() -> 2.0),
                    new SetShoulderAngle(() -> 40.0)
                )
            )
        );
    }
}
