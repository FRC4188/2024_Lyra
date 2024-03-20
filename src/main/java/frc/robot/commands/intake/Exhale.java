package frc.robot.commands.intake;

import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class Exhale extends Command {
  private Intake intake = Intake.getInstance();

  /** Creates a new Roll. */
  public Exhale() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.set(-1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
