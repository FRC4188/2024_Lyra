package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import edu.wpi.first.wpilibj2.command.Command;

public class RaiseClimber extends Command {
  private Climber climber = Climber.getInstance();

  /** Creates a new SetClimberHeight. */
  public RaiseClimber() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
