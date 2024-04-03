package frc.robot.subsystems.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.field.Goal;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

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
  private Translation3d speakerLocation = Constants.field.BLUE_SPEAKER_LOCATION;

  private InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();



  /** Creates a new Sensors. */
  private Sensors() {  

    // velocityMap.put(3.970525611458612, 15.0);
    // angleMap.put(3.970525611458612, -58.0);

    // velocityMap.put(8.0, 22.0);
    // angleMap.put(8.0, 65.5 - 90.0 + Math.toDegrees(Math.atan(2.04 / 8.0)));

    velocityMap.put(5.6, 19.2);
    angleMap.put(5.6, 58.7 - 90.0 + Math.toDegrees(Math.atan(2.04 / 5.6)));

    velocityMap.put(4.7, 19.2);
    angleMap.put(4.7, 59.2 - 90.0 + Math.toDegrees(Math.atan(2.04 / 4.7)));

    angleMap.put(4.4, 59.5 - 90.0 + Math.toDegrees(Math.atan(2.04 / 4.4)));

    velocityMap.put(3.95, 19.0);
    angleMap.put(3.95, 57.2 - 90.0 + Math.toDegrees(Math.atan(2.04 / 3.95)));

    velocityMap.put(3.7, 17.5);
    angleMap.put(3.7, 56.5 - 90.0 + Math.toDegrees(Math.atan(2.04 / 3.7)));

    velocityMap.put(3.4, 17.0);
    angleMap.put(3.4, 54.7 - 90.0 + Math.toDegrees(Math.atan(2.04 / 3.4)));

    velocityMap.put(2.9, 16.5);
    angleMap.put(2.9, 52.5 - 90.0 + Math.toDegrees(Math.atan(2.04 / 2.9)));

    velocityMap.put(2.07, 12.5);
    angleMap.put(2.07, 43.0 - 90.0 + Math.toDegrees(Math.atan(2.04 / 2.07)));

    angleMap.put(1.7, 37.0 - 90.0 + Math.toDegrees(Math.atan(2.04 / 1.7)));

    velocityMap.put(1.500, 13.0);
    angleMap.put(1.500, 32.5 - 90.0 + Math.toDegrees(Math.atan(2.04 / 1.5)));

  }

  @Override
  public void periodic() {
        
    setSpeakerLocation();  

    // SmartDashboard.putString("back ll pose", getPose2d().toString());
    SmartDashboard.putNumber("Goal XYDistance", getXYDistance());

    SmartDashboard.putNumber("Drive Angle", Swerve.getInstance().getPose2d().getRotation().getDegrees());
    SmartDashboard.putNumber("Pigeon Angle", getRotation2d().getDegrees());
    // SmartDashboard.putString("back ll pose", getBackPose2d().toString());
    // SmartDashboard.putString("front ll pose", getFrontPose2d().toString());

    SmartDashboard.putBoolean("Shooter Ready?", Shooter.getInstance().atMPS());
    SmartDashboard.putBoolean("Shoulder Ready?", Shoulder.getInstance().atGoal(getFormulaShoulderAngle()));
    SmartDashboard.putBoolean("Drive Ready?", Swerve.getInstance().atGoalAngle(getFormulaDriveAngle()));

    SmartDashboard.putNumber("Shoulder ITM Goal", Sensors.getInstance().getFormulaShoulderAngle().getDegrees());
    SmartDashboard.putNumber("Shooter ITM Goal", Sensors.getInstance().getFormulaShooterRPM());
    // SmartDashboard.putBoolean("Is Happy?", isHappy());

  }

  public Pose2d getBackPose2d() {
    if (limelightBack.getTV()) return limelightBack.getPose2d();
    return new Pose2d(); 
  }

  public Pose2d getFrontPose2d() {
    if (limelightFront.getTV()) return limelightFront.getPose2d();
    return new Pose2d(); 
  }

  public double getBackLatency() {
    return limelightBack.getLatency();
  }

  public double getFrontLatency() {
    return limelightFront.getLatency();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(pigeon.getRotation());
  }

  public double getPitch() {
    return pigeon.getPitchAsDouble();
  }

  public void resetPigeon() {
    pigeon.reset();
  }

  public void resetPigeon(Rotation2d rotation) {
    pigeon.reset(rotation);
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
    return speakerLocation.toTranslation2d().getDistance(Swerve.getInstance().getPose2d().getTranslation());
  }

  /**
   * Get the RPM that shooter should shoot at based on goal position
   * @param goal
   * @return 
   */
  public double getFormulaShooterRPM() {
    // return currentGoal.ITM_V.get(getXYDistance());
    return velocityMap.get(getXYDistance());
  }

  /**
   * Get angle the shoulder should be at while aiming + standing still 
   * @param goal
   * @return angle in double
   */
  public Rotation2d getFormulaShoulderAngle() {
    // return Rotation2d.fromDegrees(currentGoal.ITM_A.get(getXYDistance()));
    return Rotation2d.fromDegrees(-(90.0 - Math.toDegrees(Math.atan(2.04 / getXYDistance())) + angleMap.get(getXYDistance())));
  }

  /**
   * Get angle the robot / drivetrain should be at while standing still during aimmode
   * @param goal
   * @return angle in double
   */
  public Rotation2d getFormulaDriveAngle() {
    Translation2d translation = Swerve.getInstance().getPose2d().getTranslation().minus(speakerLocation.toTranslation2d());
    // Translation2d translation = speakerLocation.toTranslation2d().minus(Swerve.getInstance().getPose2d().getTranslation());

    Rotation2d setpoint = Rotation2d.fromRadians(Math.atan2(translation.getY(), translation.getX())).rotateBy(Rotation2d.fromDegrees(-3.5));
    SmartDashboard.putNumber("Drive Setpoint", setpoint.getDegrees());
    return setpoint;
  }

  public Rotation2d getFormulaDriveAngle(Translation2d goal) {
    Translation2d translation = Swerve.getInstance().getPose2d().getTranslation().minus(goal);
    // Translation2d translation = speakerLocation.toTranslation2d().minus(Swerve.getInstance().getPose2d().getTranslation());

    Rotation2d setpoint = Rotation2d.fromRadians(Math.atan2(translation.getY(), translation.getX())).rotateBy(Rotation2d.fromDegrees(-4.5));
    SmartDashboard.putNumber("Drive Setpoint", setpoint.getDegrees());
    return setpoint;
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

  public Alliance getAllianceColor() {
    var alliance = DriverStation.getAlliance();

    return alliance.isPresent() ? alliance.get() : DriverStation.Alliance.Blue;
  }

  public void setSpeakerLocation() {
    var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        speakerLocation = 
          alliance.get() == DriverStation.Alliance.Blue ? 
          Constants.field.BLUE_SPEAKER_LOCATION :
          Constants.field.RED_SPEAKER_LOCATION;
      }
  }
}