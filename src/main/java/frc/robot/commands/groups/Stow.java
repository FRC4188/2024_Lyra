package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.shoulder.Shoulder;

public class Stow extends ParallelCommandGroup {

    private Shoulder shoulder = Shoulder.getInstance();
    /** Creates a new ShootOnReady. */
    public Stow() {
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                new SetShoulderAngle(() -> 60.0)
                    .until(() -> shoulder.atGoal(60.0)),
                new SetShoulderAngle(() -> 60.0)),
                new SetShooterMPS(() -> 0.0)
            )
        );
    }
}
