package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterMPS extends Command {
  private Flywheel flywheel = Flywheel.getInstance();

  private double velocity;

  /** Creates a new SetShooterRPM. */
  public SetShooterMPS(DoubleSupplier velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
    this.velocity = velocity.getAsDouble();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.setVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
