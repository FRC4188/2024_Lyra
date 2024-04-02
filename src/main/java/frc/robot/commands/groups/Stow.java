package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

public class Stow extends ParallelCommandGroup {

    private Shoulder shoulder = Shoulder.getInstance();
    /** Creates a new ShootOnReady. */
    public Stow() {
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                new SetShoulderAngle(() -> Constants.shoulder.HANDOFF_ANGLE)
                    .until(() -> shoulder.atGoal(Rotation2d.fromDegrees(Constants.shoulder.HANDOFF_ANGLE))),
                new SetShoulderAngle(() -> Constants.shoulder.HANDOFF_ANGLE)),
                Commands.runOnce(() -> Shooter.getInstance().setControlMode(Shooter.ControlMode.STOP))
            )
            
        );
    }
}
