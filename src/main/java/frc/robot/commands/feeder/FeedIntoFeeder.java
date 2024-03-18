package frc.robot.commands.feeder;

import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.ControlMode;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedIntoFeeder extends Command {
  private Feeder feeder = Feeder.getInstance();

  /** Creates a new Feed. */
  public FeedIntoFeeder() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.setControlMode(ControlMode.PERCENT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.setPercent(0.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setControlMode(ControlMode.STOP);
  }

  // Returns true when the beam breaker is broken (note in magazine)
  @Override
  public boolean isFinished() {
    return feeder.isBroken();
  }
}
