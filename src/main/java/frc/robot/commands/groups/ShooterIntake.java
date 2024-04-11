package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.feeder.EjectFeeder;
import frc.robot.commands.feeder.FeedIntoFeeder;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shoulder.Shoulder;

public class ShooterIntake extends ParallelCommandGroup {
    
    private Shoulder shoulder = Shoulder.getInstance();
    private Feeder feeder = Feeder.getInstance();

    /** Creates a new ShootOnReady. */
    public ShooterIntake() {
        addCommands(
            new SequentialCommandGroup(
                new SetShoulderAngle(() -> 30.0)
                    .until(() -> shoulder.atGoal(Rotation2d.fromDegrees(30.0), 2.0)),
                new ParallelCommandGroup(
                    new SetShooterMPS(() -> -10.0),
                    new SetShoulderAngle(() -> 30.0)
                ).until(() -> feeder.isBroken())
            ).andThen(
                new ParallelDeadlineGroup(
                    new WaitCommand(0.5).andThen(new EjectFeeder().until(() -> !feeder.isBroken()).andThen(new FeedIntoFeeder(1.8))),
                    new SetShooterMPS(() -> -10.0)))
            .andThen(new Stow())
        );
    }
}
