package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.feeder.FeedIntoFeeder;
import frc.robot.commands.intake.Inhale;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

public class FeedIntake extends ParallelCommandGroup {
    
    private Swerve drive = Swerve.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private Feeder feeder = Feeder.getInstance();
    private Sensors sensors = Sensors.getInstance();

    /** Creates a new ShootOnReady. */
    public FeedIntake() {
        addCommands(
            new SequentialCommandGroup(
                new SetShoulderAngle(() -> 60.0)
                    .until(() -> shoulder.atGoal(60.0)),
                new ParallelCommandGroup(
                    new SetShoulderAngle(() -> 60.0),
                    new Inhale(),
                    new FeedIntoFeeder()
                )
            )
        );
    }
}
