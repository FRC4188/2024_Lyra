package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.intake.Inhale;
import frc.robot.commands.intake.Suck;

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
            Map.entry("Inhale", new Inhale().withTimeout(1.0)),
            Map.entry("Suck", new Suck().withTimeout(1.0))
      ));
}
