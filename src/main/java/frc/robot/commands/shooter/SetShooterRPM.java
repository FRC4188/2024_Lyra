package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Flywheel;
import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterRPM extends Command {
  private Flywheel flywheel = Flywheel.getInstance();

  /** Creates a new SetShooterRPM. */
  public SetShooterRPM() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
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
