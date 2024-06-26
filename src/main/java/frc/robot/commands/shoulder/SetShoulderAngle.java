package frc.robot.commands.shoulder;

import frc.robot.subsystems.shoulder.Shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SetShoulderAngle extends Command {
  private Shoulder shoulder = Shoulder.getInstance();

  private DoubleSupplier angle;

  /** Creates a new SetShoulderAngle. */
  public SetShoulderAngle(DoubleSupplier angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulder);
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulder.setAngle(Rotation2d.fromDegrees(angle.getAsDouble()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulder.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
