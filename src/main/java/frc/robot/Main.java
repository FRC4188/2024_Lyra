// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.autoworkspace.FieldObject;
import frc.robot.autoworkspace.FieldObjectHandler;
import frc.robot.autoworkspace.PathPointsGen;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    
    Translation2d startT = new Translation2d(0, 0);
    Translation2d endT = new Translation2d(1.5, 1.5);

    Pose2d startP = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d endP = new Pose2d(1.5, 1.5, new Rotation2d(0));

    FieldObject fobj = new FieldObject(0.75, 0.75, 0.2);
    FieldObjectHandler objhandler = new FieldObjectHandler(fobj);
    PathPointsGen generator = new PathPointsGen(Constants.field.FIELD_WIDTH, Constants.field.FIELD_LENGTH, 0.2, objhandler);

    //generator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()), new Pose2d(1.5, 1.5, new Rotation2d()), 0.1);
    Trajectory test = generator.generateTrajectory(startP, endP, new TrajectoryConfig(Constants.drivetrain.MAX_VELOCITY / 2, Constants.drivetrain.MAX_ACCEL / 2));
    
     System.out.println("Pivots");
    for (Translation2d pose : generator.pivots) {
      System.out.println(pose.getX() + ", " + pose.getY());
    }

    //System.out.println(test);

    
    System.out.println("Trajectory points");
    for (State state : test.getStates()) {
      System.out.println(state.poseMeters.getX() + ", " + state.poseMeters.getY());
    }
    
    
    //RobotBase.startRobot(Robot::new);
  }
}
