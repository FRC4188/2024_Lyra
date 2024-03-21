package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.drivetrain.Swerve.ControlMode;
import frc.robot.Constants;
import frc.robot.sensors.Localization;

public class TeleDrive extends Command {

  Swerve drive = Swerve.getInstance();

  Supplier<Translation2d> translationSupplier, rotationSupplier;

  /** Creates a new TeleDrive. */
  public TeleDrive(Supplier<Translation2d> translationSupplier, Supplier<Translation2d> rotationSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.translationSupplier = translationSupplier;
    this.rotationSupplier = rotationSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setControlMode(ControlMode.DRIVE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = translationSupplier.get().getY() * Constants.drivetrain.MAX_VELOCITY;
    double ySpeed = translationSupplier.get().getX() * Constants.drivetrain.MAX_VELOCITY;
    double rotSpeed = -rotationSupplier.get().getX() * Constants.drivetrain.MAX_RADIANS;

    drive.setChassisSpeeds(
    ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
        xSpeed,
        ySpeed, 
        rotSpeed), 
        Localization.getRotation2d()));

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
