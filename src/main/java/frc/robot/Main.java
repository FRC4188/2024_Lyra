// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.autoworkspace.FieldObject;
import frc.robot.autoworkspace.FieldObjectHandler;
import frc.robot.autoworkspace.PathPointsGen;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    

    RobotBase.startRobot(Robot::new);
  }
}
