package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.sensors.Sensors;

public class FarSpeakerPrep extends ParallelCommandGroup {

    public FarSpeakerPrep() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetShooterMPS(() -> 12.0),
                    new SetShoulderAngle(() -> 45.0))
            )
        );
    }
}
