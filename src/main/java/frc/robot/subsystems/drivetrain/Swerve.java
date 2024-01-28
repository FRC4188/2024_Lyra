// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.Sensors;

public class Swerve extends SubsystemBase {

  private static Swerve instance = null;

  // private Sensors sensor = Sensors.getInstance();

  /**
   * Singleton Constructor for {@link Swerve}
   * @return Single instance of {@link Swerve} common to all contexts.
   */
  public static synchronized Swerve getInstance() {
    if (instance == null) instance = new Swerve();
    return instance;
  }

  private static Translation2d[] getLocations(SwerveModule... modules) {
    Translation2d[] positions = new Translation2d[modules.length];
    for(int i = 0; i < 4; i++) {
      positions[i] = modules[i].getLocation();
    }
    return positions;
  }

  private static SwerveModulePosition[] getSwerveModulePositions(SwerveModule... modules) {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = modules[i].getModulePosition();
    }
    return modulePositions;
  }

  private SwerveModule[] moduleList = {
    new SwerveModule(
      "Front Left",
      Constants.drivetrain.FL_LOCATION,
      Constants.drivetrain.DRIVE_GEARING,
      Constants.ids.FL_SPEED,
      Constants.ids.FL_ANGLE,
      Constants.ids.FL_ENCODER,
      Constants.drivetrain.FL_ZERO
    ),
    new SwerveModule(
      "Front Right",
      Constants.drivetrain.FR_LOCATION,
      Constants.drivetrain.DRIVE_GEARING,
      Constants.ids.FR_SPEED,
      Constants.ids.FR_ANGLE,
      Constants.ids.FR_ENCODER,
      Constants.drivetrain.FR_ZERO
    ),
    new SwerveModule(
      "Back Left",
      Constants.drivetrain.BL_LOCATION,
      Constants.drivetrain.DRIVE_GEARING,
      Constants.ids.BL_SPEED,
      Constants.ids.BL_ANGLE,
      Constants.ids.BL_ENCODER,
      Constants.drivetrain.BL_ZERO
    ),
    new SwerveModule(
      "Back Right",
      Constants.drivetrain.BR_LOCATION,
      Constants.drivetrain.DRIVE_GEARING,
      Constants.ids.BR_SPEED,
      Constants.ids.BR_ANGLE,
      Constants.ids.BR_ENCODER,
      Constants.drivetrain.BR_ZERO
    )
  };

  private Sensors sensors = Sensors.getInstance();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getLocations());

  private Notifier notifier = new Notifier(() -> {
    updateDashboard();
  });


  private SwerveDrivePoseEstimator odometry =
      new SwerveDrivePoseEstimator(
          kinematics,
          // sensors.getRotation2d(),
          new Rotation2d(),
          getSwerveModulePositions(moduleList),
          new Pose2d(),
          Constants.drivetrain.STATE_STD_DEVS, 
          Constants.drivetrain.VISION_STD_DEVS);


  /** Creates a new Swerve. */
  private Swerve() {
    notifier.startPeriodic(0.2);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  public void setModuleStates(SwerveModuleState... moduleStates) {
    if (moduleStates.length == moduleList.length) {
      for (int i = 0; i < moduleList.length; i++) {
          moduleList[i].setModuleState(moduleStates[i]);
      }
    }
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      moduleStates[i] = moduleList[i].getModuleState();
    }
    return moduleStates;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose2d() {
    return odometry.getEstimatedPosition();
  }

  public double getSpeed() {
    ChassisSpeeds speeds = getChassisSpeeds();

    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public void updateOdometry() {
    Pose2d pose = sensors.getPose2d();

    if (!pose.equals(new Pose2d())) {
      odometry.addVisionMeasurement(pose, sensors.getLatency());
    }

    odometry.update(
        sensors.getRotation2d(),
        getSwerveModulePositions(moduleList));
  }

  public void updateDashboard() {
    for (SwerveModule module : moduleList) {
      SmartDashboard.putNumber(module.getName() + " Angle", module.getAngleDegrees());
    }

    SmartDashboard.putString("Position", getPose2d().toString());
  }
}
