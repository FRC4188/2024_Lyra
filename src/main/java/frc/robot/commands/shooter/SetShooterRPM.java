package frc.robot.commands.shooter;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ControlMode;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterRPM extends Command {
  private Shooter shooter = Shooter.getInstance();

  private double leftVelocity;
  private double rightVelocity;

  /** Creates a new SetShooterRPM. */
  public SetShooterRPM(DoubleSupplier leftVelocity, DoubleSupplier rightVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.leftVelocity = leftVelocity.getAsDouble();
    this.rightVelocity = rightVelocity.getAsDouble();
  }

  public SetShooterRPM(DoubleSupplier velocity) {
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
    shooter.setControlMode(ControlMode.VELOCITY);
    shooter.setVelocity(leftVelocity, rightVelocity);
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
