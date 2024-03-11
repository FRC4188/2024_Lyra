package frc.robot.subsystems.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.field.Goal;
import frc.robot.Constants.shooter.DataPoints;
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

  private Goal currentGoal = Goal.SPEAKER;

  /** Creates a new Sensors. */
  private Sensors() {
    
  }

  private void init() {}

  @Override
  public void periodic() {
    SmartDashboard.putString("Front Pose", limelightFront.getPose3d().toPose2d().toString());
    SmartDashboard.putString("Back Pose", limelightBack.getPose3d().toPose2d().toString());
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

  public boolean isHappy() {
    Pose2d pose = Swerve.getInstance().getPose2d();
    Pose2d goalPos = currentGoal.position;
    Translation2d goal = goalPos.getTranslation();
    double goalAngle = goalPos.getRotation().getRadians() - Math.PI;

    double angle1 = Math.atan2(
      goal.getX() + Math.cos(goalAngle - Math.PI / 2.0) * (currentGoal.goalWidth - Constants.field.NOTE_RADIUS) - pose.getX(),
      goal.getY() + Math.sin(goalAngle - Math.PI / 2.0) * (currentGoal.goalWidth - Constants.field.NOTE_RADIUS) - pose.getY());
    
    double angle2 = Math.atan2(
      goal.getX() - Math.cos(goalAngle - Math.PI / 2.0) * (currentGoal.goalWidth - Constants.field.NOTE_RADIUS) - pose.getX(),
      goal.getY() - Math.sin(goalAngle - Math.PI / 2.0) * (currentGoal.goalWidth - Constants.field.NOTE_RADIUS) - pose.getY());

    double d1 = (angle1 - angle2 + 2 * (Math.PI / 2.0) % 4 * (Math.PI / 2.0)) - 2 * (Math.PI / 2.0);

    return d1 > currentGoal.happyZone;
  }

  public double getXYDistance() {
    return currentGoal.position.getTranslation().getDistance(Swerve.getInstance().getPose2d().getTranslation());
  }

  /**
   * Get the RPM that shooter should shoot at based on goal position
   * @param goal
   * @return 
   */
  public double getFormulaShooterRPM() {
    return currentGoal.ITM_V.get(getXYDistance());
  }

  /**
   * Get angle the shoulder should be at while aiming + standing still 
   * @param goal
   * @return angle in double
   */
  public Rotation2d getFormulaShoulderAngle() {
    return Rotation2d.fromDegrees(currentGoal.ITM_A.get(getXYDistance()));
  }

  /**
   * Get angle the robot / drivetrain should be at while standing still during aimmode
   * @param goal
   * @return angle in double
   */
  public Rotation2d getFormulaDriveAngle() {
    Translation2d translation = Swerve.getInstance().getPose2d().getTranslation().minus(currentGoal.position.getTranslation());
    return Rotation2d.fromRadians(Math.atan2(translation.getY(), translation.getX()));
  }

  /**
   * Get the vector (angle and speed) for shooting into goal when standing still
   * @param goal
   * @return vector x, y, z in Translation3d needed for aiming while standing still
   */
  public Translation3d getShotVector() {
    double velocity = getFormulaShooterRPM();
    double shoulderAngle = getFormulaShoulderAngle().getRadians();
    double driveAngle = getFormulaDriveAngle().getRadians();

    double xy = velocity * Math.cos(shoulderAngle); // xy vector of shoulder/swerve 
    double x = xy * Math.cos(driveAngle); // x vector of swerve
    double y = xy * Math.sin(driveAngle); // y vector of swerve
    double z = velocity * Math.sin(shoulderAngle); // z vector/angle of shoulder 
    return new Translation3d(x, y, z);
  }

  /**
   * Get the vector (angle and speed) for shooting into goal while moving
   * @param goal
   * @return vector x, y, z in Translation3d needed for aiming while moving
   */
  public Translation3d getMovingShotVector() {
    Translation2d speed = Swerve.getInstance().getFOSpeeds();
    double xSpeed = speed.getX(); 
    double ySpeed = speed.getY();

    Translation3d stillShotVector = getShotVector();

    //new moving vectors need to get xy speed instead of just angles 
    return new Translation3d(stillShotVector.getX() - xSpeed, stillShotVector.getY() - ySpeed, stillShotVector.getZ());
  }

  /**
   * Get horizontal angle that serve should turn to while aiming and moving 
   * @param goal 
   * @return swerve rotation needed for aiming while moving
   */
  public Rotation2d getMovingDriveAngle() {
    Translation3d movingShotVector = getMovingShotVector();

    //get the radians of the arctan of the slope of y and x of moving vector in radian
    return Rotation2d.fromRadians(Math.atan2(movingShotVector.getY(), movingShotVector.getX()));
  }

  /**
   * Get vertical angle that shoulder should turn to while aiming and moving
   * @param goal
   * @return shoulder rotation
   */
  public Rotation2d getMovingShoulderAngle() {
    Translation3d movingShotVector = getMovingShotVector();

    double xy = Math.atan2(movingShotVector.getY(), movingShotVector.getX());
    //get the radians of the arctan of the slope of vertical and horizontal component
    return Rotation2d.fromRadians(Math.atan2(movingShotVector.getZ(), xy));
  }


  /**
   * get the rpm that the shooter should shoot at based on robot's translation position
   * @param goal
   * @return rpm in double 
   */
  public double getMovingShooterRPM() {
    Translation3d movingShotVector = getMovingShotVector();
    return movingShotVector.getNorm(); 
  }

  public void setGoal(Goal goal) {
    currentGoal = goal;
  }

  // public void putITMData() {
  //   currentGoal.ITM_A.put(0.0, 8.0);
  // }
}