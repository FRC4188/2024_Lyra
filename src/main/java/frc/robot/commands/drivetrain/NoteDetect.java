package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import javax.transaction.xa.Xid;

import com.fasterxml.jackson.databind.ser.std.NumberSerializers.DoubleSerializer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Limelight;
import frc.robot.subsystems.sensors.Sensors;

public class NoteDetect extends Command {
    double noteAngle = 0.0;
    boolean noInput;
    Swerve drive = Swerve.getInstance();
    PIDController turnPID = new PIDController(SmartDashboard.getNumber("rot P", 0.0), 0.0, 0.006); //from rot pid constants
    Swerve swerve = Swerve.getInstance();
    Limelight detectingLimelight =
      new Limelight(
          "limelight-front",
          Constants.sensors.limelight.FRONT_LIMELIGHT_LOCATION);
    DoubleSupplier xInput, yInput;
    // Called when the command is initially scheduled.
    @Override
        public void initialize() {
    }

    public NoteDetect(DoubleSupplier xInput, DoubleSupplier yInput){
        this.xInput = xInput;
        this.yInput = yInput;
    }
    @Override
    public void execute() {
        noteAngle = detectingLimelight.getTX();
        // if (detectingLimelight.getTV()){
        //     turnPID.calculate(swerve.getPose2d().getRotation().getDegrees(), noteAngle);
        //     SmartDashboard.putNumber("note to crosshair", noteAngle);
        // }
double totalSpeed = Math.pow(Math.hypot(xInput.getAsDouble(), yInput.getAsDouble()), 3.0);
    //arctan of the slope of y and x = angle
    double angle = Math.atan2(yInput.getAsDouble(), xInput.getAsDouble());

    double xSpeed = totalSpeed * Math.cos(angle) * Constants.drivetrain.MAX_VELOCITY;
    double ySpeed = totalSpeed * Math.sin(angle) * Constants.drivetrain.MAX_VELOCITY;
    double rotSpeed = 0.0;

    //rotSpeed calculated from rotPID
    rotSpeed += drive.rotPID.calculate(Swerve.getInstance().getPose2d().getRotation().getDegrees(), noteAngle);
    /**rotSpeed from aiden's math hellscape = predicting wut the drivetrain rotation speed should be
     * to keep aiming while moving */
    // rotSpeed += Math.toDegrees(currentSpeed.getX() * -dy / (dx * dx + dy * dy) + currentSpeed.getY() * dx / (dx * dx + dy * dy));

    noInput = xSpeed == 0 && ySpeed == 0 && rotSpeed == 0;

    //make the swerve form "X" to prevent drifting when stop
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
          xSpeed,
          ySpeed, 
          rotSpeed), 
          Sensors.getInstance().getRotation2d()));
    }    }
    // public double distance() {
    //     return noteAngle;
    // }
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  
}
