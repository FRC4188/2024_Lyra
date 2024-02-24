package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.feeder.Feed;
import frc.robot.commands.shooter.SetShooterRPM;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

public class BlindShoot extends ParallelCommandGroup {
    Shoulder shoulder = Shoulder.getInstance();
    Shooter shooter = Shooter.getInstance();

    public BlindShoot() {

        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetShooterRPM(() -> 0.0),
                    new SetShoulderAngle(() -> 70.0)
                ).until(() -> shoulder.atGoal(70.0) && shooter.atRPM(0.0)),
                new Feed()
            )
        );
    }
}
