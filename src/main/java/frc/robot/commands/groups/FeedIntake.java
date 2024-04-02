package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.feeder.FeedIntoFeeder;
import frc.robot.commands.feeder.Heimlich;
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
                    .until(() -> shoulder.atGoal(Rotation2d.fromDegrees(Constants.shoulder.HANDOFF_ANGLE), 5.0)),
                new ParallelDeadlineGroup(
                    new FeedIntoFeeder(3.0).andThen(new Heimlich()),
                    new SetShoulderAngle(() -> Constants.shoulder.HANDOFF_ANGLE),
                    new Inhale()
                ),
                new ParallelDeadlineGroup(
                    new Heimlich(),
                    Commands.run(() -> RobotContainer.pilot.setRumble(RumbleType.kBothRumble, 1.0)),
                    Commands.run(() -> RobotContainer.copilot.setRumble(RumbleType.kBothRumble, 1.0))
                        
                ).andThen(
                    new ParallelCommandGroup(
                        Commands.run(() -> RobotContainer.pilot.setRumble(RumbleType.kBothRumble, 0.0)),
                        Commands.run(() -> RobotContainer.copilot.setRumble(RumbleType.kBothRumble, 0.0))
                    )
                )
            )
        );
    }
}
