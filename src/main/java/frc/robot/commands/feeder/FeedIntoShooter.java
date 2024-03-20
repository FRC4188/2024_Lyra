package frc.robot.commands.feeder;

import frc.robot.subsystems.feeder.Feeder;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedIntoShooter extends Command {
  private Feeder feeder = Feeder.getInstance();
  private double voltage;

  /** Creates a new Feed. */
  public FeedIntoShooter(double voltage) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
    this.voltage = voltage;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.setVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
