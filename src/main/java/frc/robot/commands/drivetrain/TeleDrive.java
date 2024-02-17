package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.Constants;

public class TeleDrive extends Command {

  Swerve drive = Swerve.getInstance();
  Sensors sensors = Sensors.getInstance();

  DoubleSupplier xInput, yInput, thetaInput;
  boolean noInput;
  /** Creates a new TeleDrive. */
  public TeleDrive(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.xInput = xInput;
    this.yInput = yInput;
    this.thetaInput = thetaInput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    noInput = xInput.getAsDouble() == 0 && yInput.getAsDouble() == 0 && thetaInput.getAsDouble() == 0;

    if (noInput) {
      drive.setModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))});
    } else {
      drive.setChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
          xInput.getAsDouble() * Constants.drivetrain.MAX_VELOCITY,
          -yInput.getAsDouble() * Constants.drivetrain.MAX_VELOCITY, 
          -thetaInput.getAsDouble() * Constants.drivetrain.MAX_RADIANS), 
          sensors.getRotation2d()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
