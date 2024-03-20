package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;

public class BlindAmpPrep extends ParallelCommandGroup {

    public BlindAmpPrep() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetShooterMPS(() ->3.5),
                    new SetShoulderAngle(() -> 37.0))
            )
        );
    }
}