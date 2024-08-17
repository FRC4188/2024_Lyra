package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Limelight;

public class NoteDetect extends Command {
    double noteAngle = 0.0;
    PIDController turnPID = new PIDController(0.15, 0.0, 0.006); //from rot pid constants
    Swerve swerve = Swerve.getInstance();
    Limelight detectingLimelight =
      new Limelight(
          "limelight-front",
          Constants.sensors.limelight.FRONT_LIMELIGHT_LOCATION);
    // Called when the command is initially scheduled.
    @Override
        public void initialize() {
    }

    @Override
    public void execute() {
        noteAngle = detectingLimelight.getTX(detectingLimelight);
        // if (detectingLimelight.getTV()){
        //     turnPID.calculate(swerve.getPose2d().getRotation().getDegrees(), noteAngle);
        //     SmartDashboard.putNumber("note to crosshair", noteAngle);
        // }
        new TrackingDrive(() -> 0.0, ()-> 0.0, () -> noteAngle); //the x or y (i think xcomponent will probably be changed
    }
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
