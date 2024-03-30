package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
  boolean input;

  SlewRateLimiter limiterX;
  SlewRateLimiter limiterY;

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
    limiterX = new SlewRateLimiter(Constants.drivetrain.MAX_ACCEL);
    limiterY = new SlewRateLimiter(Constants.drivetrain.MAX_ACCEL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double totalSpeed = Math.pow(Math.hypot(xInput.getAsDouble(), yInput.getAsDouble()), 1.0);
    double angle = Math.atan2(yInput.getAsDouble(), xInput.getAsDouble());
    double xSpeed = totalSpeed * Math.cos(angle) * Constants.drivetrain.MAX_VELOCITY;
    double ySpeed = totalSpeed * Math.sin(angle) * Constants.drivetrain.MAX_VELOCITY;
    double rotSpeed = -thetaInput.getAsDouble() * Constants.drivetrain.MAX_RADIANS;

    xSpeed = limiterX.calculate(xSpeed);
    ySpeed = limiterY.calculate(ySpeed);

    // noInput = xSpeed == 0 && ySpeed == 0 && rotSpeed == 0;

    // if (noInput) {
    //   drive.setModuleStates(
    //     new SwerveModuleState[] {
    //       new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    //       new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
    //       new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
    //       new SwerveModuleState(0, Rotation2d.fromDegrees(45))});
    // } else {
      drive.setChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
          xSpeed,
          ySpeed, 
          rotSpeed), 
          drive.getPose2d().getRotation()));

    input = xInput.getAsDouble() != 0.0 || yInput.getAsDouble() != 0.0;

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
