// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import java.lang.invoke.ConstantBootstraps;

import CSP_Lib.utils.LimelightHelpers;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;

/** Add your docs here. */
public class Limelight {
  private String name;
  private Translation3d position = new Translation3d();
  private Rotation3d rotation = new Rotation3d();

  private Pose2d currentPose = new Pose2d();

  // private MedianFilter filter = new MedianFilter(2);

  public Limelight(String name, Translation3d position, Rotation3d rotation) {
    this.name = name;
    this.position = position;
    this.rotation = rotation;
    init();
  }

  private void init() {
    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        position.getX(),
        position.getY(),
        position.getZ(),
        Units.radiansToDegrees(rotation.getX()),
        Units.radiansToDegrees(rotation.getY()),
        Units.radiansToDegrees(rotation.getZ()));
  }

  public Pose2d getPose2d() {
    if (LimelightHelpers.getTV(name)) {
      currentPose = LimelightHelpers.getBotPose3d_wpiBlue(name).toPose2d();
      return currentPose;
    } else {
      return new Pose2d();
    }
  }
  
  public boolean getTV() {
    return LimelightHelpers.getTV(name);
  }

  public double getLatency() {
    double time = Timer.getFPGATimestamp();
    if (LimelightHelpers.getTV(name)) {
      return time
          - (LimelightHelpers.getLatency_Capture(name) / 1000)
          - (LimelightHelpers.getLatency_Pipeline(name) / 1000);
    } else return 0.0;
  }

  public double getAvgTagDistance() {
    return getTV() ? LimelightHelpers.getBotPose(name)[9] : 0;
  }

  // public Matrix<N1, N3> getConfidence() {
  //       // distance from current pose to vision estimated pose
    

  //   if (getTV()) {
  //     double xyStds, degStds;
  //     double poseDifference = Swerve.getInstance().getPose2d().getTranslation().getDistance(getPose2d().getTranslation());
  //     double[] botpose = LimelightHelpers.getBotPose(name);
  //     // multiple targets detected
  //     if (botpose[7] >= 2) {
  //       xyStds = 0.5;
  //       degStds = 6;
  //     }
  //     // 1 target with large area and close to estimated pose
  //     else if (botpose[10] > 0.8 && poseDifference < 0.5) {
  //       xyStds = 1.0;
  //       degStds = 12;
  //     }
  //     // 1 target farther away and estimated pose is close
  //     else if (botpose[10] > 0.1 && poseDifference < 0.3) {
  //       xyStds = 2.0;
  //       degStds = 30;
  //     } else {
  //       xyStds = 5.0;
  //       degStds = 50;
  //     }

  //     return new Matrix<>
      
  //   }
  // }

}
