package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.groups.FeedIntake;
import frc.robot.commands.groups.BlindReverseSpeakerPrep;
import frc.robot.commands.groups.Stow;
import frc.robot.commands.intake.Inhale;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;

/** Add your docs here. */
public class AutoConfigs {
  public static final PathConstraints DEFAULT_CONSTRAINTS =
      new PathConstraints(Constants.drivetrain.MAX_VELOCITY, Constants.drivetrain.MAX_ACCEL, 1, 1);

  public static final HashMap<String, Command> EVENTS =
      new HashMap<>(
          Map.ofEntries(
            // Map.entry("Name of Command", new Command()),
            // Map.entry("Name of Command", new Command().withTimeout(TIME_SECONDS))
            Map.entry("Blind Intake", new FeedIntake()),
            Map.entry("Auto Speaker Shoot", new ParallelDeadlineGroup(new BlindReverseSpeakerPrep().withTimeout(1.0), Commands.waitSeconds(0.5).andThen(new FeedIntoShooter(12.0))))
      ));
}
