package frc.robot.commands.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class TrackingDrive extends Command {
  private Swerve drive = Swerve.getInstance();

  /** Creates a new TrackingDrive. */
  public TrackingDrive(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
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
