package frc.robot.commands.feeder;

import frc.robot.subsystems.feeder.Feeder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ReturnToFeeder extends Command {
  private Feeder feeder = Feeder.getInstance();

  /** Creates a new Feed. */
  public ReturnToFeeder() {
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
    feeder.set(-0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.set(0.0);
  }

  // Command ends when beam breaker is not broken (when note shot out)
  @Override
  public boolean isFinished() {
    return false;
  }
}
