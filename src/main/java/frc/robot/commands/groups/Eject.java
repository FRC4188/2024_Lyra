package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.feeder.EjectFeeder;
import frc.robot.commands.intake.Exhale;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.shoulder.Shoulder;

public class Eject extends ParallelCommandGroup {
    
    private Shoulder shoulder = Shoulder.getInstance();

    /** Creates a new ShootOnReady. */
    public Eject() {
        addCommands(
            new SequentialCommandGroup(
                new SetShoulderAngle(() -> Constants.shoulder.HANDOFF_ANGLE)
                    .until(() -> shoulder.atGoal(Rotation2d.fromDegrees(Constants.shoulder.HANDOFF_ANGLE))),
                new ParallelCommandGroup(
                    new EjectFeeder(),
                    new SetShoulderAngle(() -> Constants.shoulder.HANDOFF_ANGLE),
                    new Exhale()
                )
            )
        );
    }
}
