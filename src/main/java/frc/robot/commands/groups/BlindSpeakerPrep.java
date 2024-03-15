package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;

public class BlindSpeakerPrep extends ParallelCommandGroup {

    public BlindSpeakerPrep() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetShooterMPS(() -> 13.0),
                    new SetShoulderAngle(() -> 32.5))
            )
        );
    }
}