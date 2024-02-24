package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.feeder.Feed;
import frc.robot.commands.intake.Inhale;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shoulder.Shoulder;

public class FeedShooter extends ParallelCommandGroup {
    
    private Swerve drive = Swerve.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private Flywheel shooter = Flywheel.getInstance();
    private Feeder feeder = Feeder.getInstance();
    private Sensors sensors = Sensors.getInstance();

    /** Creates a new ShootOnReady. */
    public FeedShooter() {
        addCommands(
            new SequentialCommandGroup(
                new SetShoulderAngle(() -> 70.0)
                    .until(() -> shoulder.atGoal(70.0)),
                new ParallelCommandGroup(
                    new SetShoulderAngle(() -> 70.0),
                    new Inhale()
                )
            )
        );
    }


}
