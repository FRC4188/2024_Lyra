package frc.robot.commands.feeder;

import frc.robot.subsystems.feeder.Feeder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
