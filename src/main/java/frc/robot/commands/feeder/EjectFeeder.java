package frc.robot.commands.feeder;

import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.ControlMode;
import edu.wpi.first.wpilibj2.command.Command;

public class EjectFeeder extends Command {
  private Feeder feeder = Feeder.getInstance();

  /** Creates a new Feed. */
  public EjectFeeder() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.setControlMode(ControlMode.PERCENT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.setPercent(-0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setControlMode(ControlMode.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
