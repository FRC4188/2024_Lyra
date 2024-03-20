package frc.robot.commands.feeder;

import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Heimlich extends ParallelCommandGroup {

    /** Creates a new Heimlich. 
     * 
    */
    public Heimlich() {
        addCommands(
            new SequentialCommandGroup(
                Commands.waitUntil(() -> Feeder.getInstance().isBroken()),
                new EjectFeeder().until(() -> !Feeder.getInstance().isBroken()),
                new FeedIntoFeeder(1.8)
            )
        );
    }
}
