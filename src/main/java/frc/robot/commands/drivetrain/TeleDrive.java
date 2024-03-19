package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.drivetrain.Swerve.ControlMode;
import frc.robot.Constants;
import frc.robot.sensors.Localization;

public class TeleDrive extends Command {

  Swerve drive = Swerve.getInstance();

  DoubleSupplier xInput, yInput, thetaInput;
  boolean noInput;

  SlewRateLimiter limiterX = new SlewRateLimiter(Constants.drivetrain.MAX_ACCEL * 2.0);
  SlewRateLimiter limiterY = new SlewRateLimiter(Constants.drivetrain.MAX_ACCEL * 2.0);

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
  public void initialize() {
    drive.setControlMode(ControlMode.DRIVE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double totalSpeed = Math.pow(Math.hypot(xInput.getAsDouble(), yInput.getAsDouble()), 1.0);
    double angle = Math.atan2(yInput.getAsDouble(), xInput.getAsDouble());
    double xSpeed = totalSpeed * Math.cos(angle) * Constants.drivetrain.MAX_VELOCITY;
    double ySpeed = totalSpeed * Math.sin(angle) * Constants.drivetrain.MAX_VELOCITY;
    double rotSpeed = -thetaInput.getAsDouble() * Constants.drivetrain.MAX_RADIANS;

    xSpeed = Math.signum(xSpeed) * limiterX.calculate(Math.abs(xSpeed));
    ySpeed = Math.signum(ySpeed) * limiterY.calculate(Math.abs(ySpeed));

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
