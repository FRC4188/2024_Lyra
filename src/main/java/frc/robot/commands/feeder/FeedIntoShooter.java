package frc.robot.commands.feeder;

import frc.robot.Constants;
import frc.robot.subsystems.feeder.Feeder;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedIntoShooter extends Command {
  private Feeder feeder = Feeder.getInstance();

  /** Creates a new Feed. */
  public FeedIntoShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.set(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !feeder.isBroken();
  }
}
