// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Sensors;

public class Swerve extends SubsystemBase {

  private static Swerve instance = null;
  /**
   * Singleton Constructor for {@link Swerve}
   * @return Single instance of {@link Swerve} common to all contexts.
   */
  public static synchronized Swerve getInstance() {
    if (instance == null) instance = new Swerve();
    return instance;
  }

  private SwerveModule frontRight;
  private SwerveModule frontLeft;
  private SwerveModule backLeft;
  private SwerveModule backRight;

  private Sensors sensors = Sensors.getInstance();

  private SwerveDrivePoseEstimator odometry;
  private SwerveDriveKinematics kinematics;
  
  /** Creates a new Swerve. */
  public Swerve() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
