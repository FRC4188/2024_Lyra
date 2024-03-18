// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  private static Swerve instance = null;

  /**
   * Singleton Constructor for {@link Swerve}
   * @return Single instance of {@link Swerve} common to all contexts <b>(Should always be kept static)</b>.
   */
  public static synchronized Swerve getInstance() {
    if (instance == null) instance = new Swerve();
    return instance;
  }

  private enum ControlMode {
    STOP,
    X_PATTERN,
    FIELD_ORIENTED,
    ROBOT_ORIENTED
  }
  private ControlMode mode = ControlMode.STOP;

  private Limiter limiter = new CompetitionLimit();

  /** Creates a new Swerve. */
  private Swerve() {
  }

  @Override
  public void periodic() {

    switch (mode) {

      case STOP:
        
        break;
    
      case FIELD_ORIENTED:

        break;
      
      case ROBOT_ORIENTED:

        break;

      default:

        break;
    }
 }

}