package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.drivetrain.Swerve.ControlMode;
import frc.robot.Constants;

public class HockeyStop extends Command {

  Swerve drive = Swerve.getInstance();
 
  public HockeyStop() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  ChassisSpeeds speeds;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setControlMode(ControlMode.MODULE_STATES);
    speeds = drive.getChassisSpeeds();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (speeds.vxMetersPerSecond != 0.0 || speeds.vyMetersPerSecond != 0.0) {
        double angle = Units.radiansToDegrees(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond)) + 90.0;
        angle = (angle + 180.0) % 360.0 - 180.0;

            drive.setModuleStates(
        new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(angle)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(angle)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(angle)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(angle))});
    } else {
        drive.setControlMode(ControlMode.X_PATTERN);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setControlMode(ControlMode.X_PATTERN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
