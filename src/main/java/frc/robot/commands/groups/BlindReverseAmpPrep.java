package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;

public class BlindReverseAmpPrep extends ParallelCommandGroup {

    public BlindReverseAmpPrep() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetShooterMPS(() -> 3.3),
                    new SetShoulderAngle(() -> -33.0))
            )
        );
    }
}