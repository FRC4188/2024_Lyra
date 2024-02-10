package frc.robot.commands.shoulder;

import frc.robot.Constants;
import frc.robot.subsystems.shoulder.Shoulder;
import edu.wpi.first.wpilibj2.command.Command;

public class SetShoulderAngle extends Command {
  private Shoulder shoulder = Shoulder.getInstance();

  /** Creates a new SetShoulderAngle. */
  public SetShoulderAngle() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulder);
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
