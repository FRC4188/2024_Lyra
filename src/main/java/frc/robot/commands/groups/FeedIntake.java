package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.feeder.FastFeeder;
import frc.robot.commands.feeder.FeedIntoFeeder;
import frc.robot.commands.intake.Inhale;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.shoulder.Shoulder;

public class FeedIntake extends ParallelCommandGroup {
    
    private Shoulder shoulder = Shoulder.getInstance();

    /** Creates a new ShootOnReady. */
    public FeedIntake() {
        addCommands(
            new SequentialCommandGroup(
                new SetShoulderAngle(() -> Constants.shoulder.HANDOFF_ANGLE)
                    .until(() -> shoulder.atGoal(Constants.shoulder.HANDOFF_ANGLE)),
                new ParallelDeadlineGroup(
                    new FastFeeder(),
                    new SetShoulderAngle(() -> Constants.shoulder.HANDOFF_ANGLE),
                    new Inhale()
                )
            )
        );
    }
}
