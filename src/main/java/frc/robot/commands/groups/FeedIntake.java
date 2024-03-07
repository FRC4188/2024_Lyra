package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.feeder.FeedIntoFeeder;
import frc.robot.commands.intake.Inhale;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shoulder.Shoulder;

public class FeedIntake extends ParallelCommandGroup {
    
    private Shoulder shoulder = Shoulder.getInstance();
    private Intake intake = Intake.getInstance();

    /** Creates a new ShootOnReady. */
    public FeedIntake() {
        addCommands(
            new SequentialCommandGroup(
                new Inhale().until(() -> intake.isBroken()),
                new SetShoulderAngle(() -> 60.0)
                    .until(() -> shoulder.atGoal(60.0)),
                new ParallelDeadlineGroup(
                    new FeedIntoFeeder(),
                    new SetShoulderAngle(() -> 60.0),
                    new Inhale()
                )
            )
        );
    }
}
