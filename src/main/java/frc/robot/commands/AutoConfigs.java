package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.groups.BlindReverseSpeakerShoot;
import frc.robot.commands.groups.FeedIntake;
import frc.robot.commands.groups.ShootOnReady;
import frc.robot.commands.groups.Stow;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.feeder.Feeder;

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
            Map.entry("Auto Speaker Shoot", new BlindReverseSpeakerShoot().withTimeout(1.2)),
            Map.entry("Shooter Prep", new SetShooterMPS(() -> 13.0)),
            Map.entry("Shoot on Ready", new ConditionalCommand(
                new ShootOnReady(() -> 0.0, () -> 0.0).withTimeout(1.5)
                                            .andThen(
                                                new ConditionalCommand(
                                                    new FeedIntoShooter(12.0).withTimeout(0.9),
                                                    new SequentialCommandGroup(),
                                                    () -> Feeder.getInstance().isBroken())
                                            ),
                new FeedIntake().withTimeout(1.5).andThen(
                new ShootOnReady(() -> 0.0, () -> 0.0).withTimeout(1.5)
                                            .andThen(
                                                new ConditionalCommand(
                                                    new FeedIntoShooter(12.0).withTimeout(0.9),
                                                    new SequentialCommandGroup(),
                                                    () -> Feeder.getInstance().isBroken())
                                            )),
                () -> Feeder.getInstance().isBroken()
            ).withTimeout(4.0))
            
            ,
            Map.entry(
                  "Stop Drivetrain",
                  new InstantCommand(
                      () -> Swerve.getInstance().disable(), Swerve.getInstance())),
            Map.entry("Stow", new Stow().withTimeout(0.25)),
            Map.entry("Delay", new WaitCommand(2.0))
      ));
}
