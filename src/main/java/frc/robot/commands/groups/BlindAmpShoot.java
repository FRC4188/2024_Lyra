package frc.robot.commands.groups;

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
                    new SetShooterMPS(() -> 30.0),
                    new SetShoulderAngle(() -> 60.0)
                ).until(() -> shoulder.atGoal(60.0) && shooter.atRPM(30.0)), // continue after condition met
                new FeedIntoShooter()
            )
        );
    }
}
