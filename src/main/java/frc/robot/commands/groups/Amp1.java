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

public class Amp1 extends ParallelCommandGroup {
    Shoulder shoulder = Shoulder.getInstance();
    Shooter shooter = Shooter.getInstance();

    public Amp1() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetShoulderAngle(() -> 10.0),
                    new RunCommand(() -> shooter.setVelocity(3.0, 3.0)),
                                        new RunCommand(() -> shooter.setControlMode(ControlMode.VELOCITY))

                ).until(() -> shooter.atMPS(3.0) && shoulder.atGoal(Rotation2d.fromDegrees(10.0), 2.0)),
                new ParallelCommandGroup(
                    Commands.waitSeconds(0.2).andThen(new FeedIntoShooter(12.0)),
                    new RunCommand(() -> shooter.setVelocity(3.0, 3.0)),
                    new RunCommand(() -> shoulder.setVoltage(7.0)).until(() -> shoulder.atGoal(Rotation2d.fromDegrees(40.0), 2.0))
                )
            )
        );
    }
}