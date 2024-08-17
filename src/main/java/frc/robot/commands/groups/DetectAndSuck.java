package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.NoteDetect;
import frc.robot.commands.intake.Inhale;
import frc.robot.subsystems.intake.Intake;

public class DetectAndSuck extends SequentialCommandGroup{
    public DetectAndSuck() {
        addCommands(
            new NoteDetect(),
                new FeedIntake());
    }
}
