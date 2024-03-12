package frc.robot.commands.shooter;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ControlMode;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterMPS extends Command {
  private Shooter shooter = Shooter.getInstance();

  private double leftVelocity;
  private double rightVelocity;

  /** Creates a new SetShooterRPM. */
  public SetShooterMPS(DoubleSupplier leftVelocity, DoubleSupplier rightVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.leftVelocity = leftVelocity.getAsDouble();
    this.rightVelocity = rightVelocity.getAsDouble();
  }

  public SetShooterMPS(DoubleSupplier velocity) {
    addRequirements(shooter);
    this.leftVelocity = velocity.getAsDouble();
    this.rightVelocity = velocity.getAsDouble();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocity(leftVelocity, rightVelocity);
    shooter.setControlMode(ControlMode.VELOCITY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.disable();
    shooter.setControlMode(ControlMode.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
