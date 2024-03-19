package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ShotConstants.BlindShots;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.sensors.Localization;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

public class BlindShoot extends ParallelCommandGroup {
    Shoulder shoulder = Shoulder.getInstance();
    Shooter shooter = Shooter.getInstance();
    double angle;

    public BlindShoot(BlindShots shot) {
        angle = shot.getAngle();
        if(Localization.isFlipped()) angle *= -1;

        addCommands(
            new ParallelDeadlineGroup(
                    Commands.waitUntil(() -> 
                        shooter.atRPM(shot.getVelocity()) && 
                        shoulder.atGoal(angle)).andThen(
                    new FeedIntoShooter().andThen(Commands.waitSeconds(0.25)),
                    new SetShooterMPS(() -> shot.getVelocity()),
                    new SetShoulderAngle(() -> angle)
                )
            )
        );
    }
}
