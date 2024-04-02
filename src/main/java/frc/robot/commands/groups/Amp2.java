package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ControlMode;
import frc.robot.subsystems.shoulder.Shoulder;

public class Amp2 extends ParallelCommandGroup {
    Shoulder shoulder = Shoulder.getInstance();
    Shooter shooter = Shooter.getInstance();

    public Amp2() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetShoulderAngle(() -> 20.0),
                    new RunCommand(() -> shooter.setVelocity(3.0, 3.0)),
                                        new RunCommand(() -> shooter.setControlMode(ControlMode.VELOCITY))

                ).until(() -> shooter.atMPS(3.0) && shoulder.atGoal(Rotation2d.fromDegrees(20.0), 1.5)),
                new ParallelCommandGroup(
                    Commands.waitSeconds(0.2).andThen(new FeedIntoShooter(12.0)),
                    new RunCommand(() -> shooter.setVelocity(3.0, 3.0)),
                    new SetShoulderAngle(() -> 40.0)
                )
            )
        );
    }
}
