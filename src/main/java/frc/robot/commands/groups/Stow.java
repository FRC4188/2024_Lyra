package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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
                new SetShoulderAngle(() -> Constants.shoulder.HANDOFF_ANGLE)
                    .until(() -> shoulder.atGoal(Constants.shoulder.HANDOFF_ANGLE)),
                new SetShoulderAngle(() -> Constants.shoulder.HANDOFF_ANGLE)),
                new SetShooterMPS(() -> 0.0)
            )
        );
    }
}
