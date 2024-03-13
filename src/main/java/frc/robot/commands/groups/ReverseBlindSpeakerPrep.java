package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;

public class ReverseBlindSpeakerPrep extends ParallelCommandGroup {
    public ReverseBlindSpeakerPrep() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetShooterMPS(() -> 10.0),
                    new SetShoulderAngle(() -> 32.5))
            )
        );
    }
}
