package frc.robot.commands.intake;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.ControlMode;
import edu.wpi.first.wpilibj2.command.Command;

public class Exhale extends Command {

  Intake intake = Intake.getInstance();

  /** Creates a new Roll. */
  public Exhale() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setPercent(-1.0);
    intake.setControlMode(ControlMode.PERCENT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setControlMode(ControlMode.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
