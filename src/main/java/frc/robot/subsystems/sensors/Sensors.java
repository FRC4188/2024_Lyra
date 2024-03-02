package frc.robot.subsystems.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.drivetrain;
import frc.robot.subsystems.drivetrain.Swerve;

public class Sensors extends SubsystemBase {

  private static Sensors instance = null;

  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  //Offset is 180 because the pigeon is oriented backwards
  private Pigeon pigeon = new Pigeon(Constants.ids.PIGEON, Constants.sensors.pigeon.PIGEON_OFFSET_DEGREES);

  private Limelight limelightFront =
      new Limelight(
          Constants.sensors.limelight.FRONT_NAME,
          Constants.sensors.limelight.FRONT_POSITION,
          Constants.sensors.limelight.FRONT_ROTATION);
  private Limelight limelightBack =
      new Limelight(
          Constants.sensors.limelight.BACK_NAME,
          Constants.sensors.limelight.BACK_POSITION,
          Constants.sensors.limelight.BACK_ROTATION);

  Swerve drive = Swerve.getInstance();

  /** Creates a new Sensors. */
  private Sensors() {}

  private void init() {}

  @Override
  public void periodic() {
    SmartDashboard.putString("Front Pose", getPose2d().toString());
    SmartDashboard.putString("Back Pose", limelightFront.getPose3d().toPose2d().toString());
    SmartDashboard.putNumber("Pigeon Angle", getRotation2d().getDegrees());
  }

  public Pose3d getPose3d() {
    if (limelightFront.getTV() && limelightBack.getTV()) {
      Pose3d poseLeft = limelightBack.getPose3d();
      Pose3d poseRight = limelightBack.getPose3d();

      // averages the poses of both limelights
      return new Pose3d(
          poseLeft.getTranslation().plus(poseRight.getTranslation()).div(2),
          poseLeft.getRotation().plus(poseRight.getRotation()).div(2));
    } else if (limelightFront.getTV()) 
        return limelightFront.getPose3d();
     else if (limelightBack.getTV()) 
        return limelightBack.getPose3d();
     else return new Pose3d(); 
  }

  public Pose2d getPose2d() {
    return getPose3d().toPose2d();
  }

  public double getLatency() {
    if (limelightFront.getTV() && limelightBack.getTV()) {
      // averages the latencies
      return (limelightFront.getLatency() + limelightBack.getLatency()) / 2;
    } else if (limelightFront.getTV()) {
      return limelightFront.getLatency();
    } else if (limelightBack.getTV()) {
      return limelightBack.getLatency();
    } else return 0.0;
  }

  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d();
  }

  public double getPitch() {
    return pigeon.getPitchAsDouble();
  }

  public void resetPigeon() {
    pigeon.reset();
  }

  public double getXYDistance(Translation3d goal) {
    return goal.toTranslation2d().getDistance(drive.getPose2d().getTranslation());
  }

  /**
   * Get the RPM that shooter should shoot at based on goal position
   * @param goal
   * @return 
   */
  public double getFormulaShooterRPM(Translation3d goal) {
    return 0;
  }

  /**
   * Get angle the shoulder should be at 
   * @param goal
   * @return angle in double
   */
  public double getFormulaShoulderAngle(Translation3d goal) {
    return Math.atan2((goal.getZ() - Constants.robot.SHOULDER_PIVOT_HEIGHT), getXYDistance(goal));
  }

  /**
   * Get angle the robot / drivetrain should be at
   * @param goal
   * @return angle in double
   */
  public double getFormulaDriveAngle(Translation3d goal) {
    Translation2d translation = drive.getPose2d().getTranslation().minus(goal.toTranslation2d());
    return Math.atan2(translation.getY(), translation.getX());
  }

  /**
   * Get the vector (angle and speed) for shooting into goal when standing still
   * @param goal
   * @return vector x, y, z in Translation3d needed for aiming while standing still
   */
  public Translation3d getShotVector(Translation3d goal) {
    double velocity = getFormulaShooterRPM(goal);
    double shoulderAngle = getFormulaShoulderAngle(goal);
    double driveAngle = getFormulaDriveAngle(goal);

    double xy = velocity * Math.cos(shoulderAngle); // xy vector of shoulder/serve 
    double x = xy * Math.cos(Units.degreesToRadians(driveAngle)); //x vector of swerve
    double y = xy * Math.sin(Units.degreesToRadians(driveAngle)); //y vector of swerve
    double z = velocity * Math.sin(Units.degreesToRadians(shoulderAngle)); //z vector/angle of shoulder 
    return new Translation3d(x, y, z);
  }

  /**
   * Get the vector (angle and speed) for shooting into goal while moving
   * @param goal
   * @return vector x, y, z in Translation3d needed for aiming while moving
   */
  public Translation3d getMovingShotVector(Translation3d goal) {
    Translation2d speed = drive.getFOSpeeds();
    double xSpeed = speed.getX();
    double ySpeed = speed.getY();

    Translation3d stillShotVector = getShotVector(goal);

    //new moving vectors need to get xy speed instead of just angles 
    return new Translation3d(stillShotVector.getX() - xSpeed, stillShotVector.getY() - ySpeed, stillShotVector.getZ());
  }

  public Rotation2d getMovingDriveAngle(Translation3d goal) {
    Translation3d movingShotVector = getMovingShotVector(goal);
    return Rotation2d.fromRadians(Math.atan2(movingShotVector.getY(), movingShotVector.getX()));
  }

  public Rotation2d getMovingShoulderAngle(Translation3d goal) {
    Translation3d movingShotVector = getMovingShotVector(goal);
    double xy = Math.atan2(movingShotVector.getY(), movingShotVector.getX());
    return Rotation2d.fromRadians(Math.atan2(movingShotVector.getZ(), xy));
  }

  public double getMovingShooterRPM(Translation3d goal) {
    Translation3d movingShotVector = getMovingShotVector(goal);
    return movingShotVector.getNorm();
  }
}