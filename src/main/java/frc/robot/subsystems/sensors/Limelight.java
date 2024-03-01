// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import CSP_Lib.utils.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class Limelight {
  private String name;
  private Translation3d position = new Translation3d();
  private Rotation3d rotation = new Rotation3d();

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

  public Pose3d getPose3d() {
    if (LimelightHelpers.getTV(name)) {
      return LimelightHelpers.getBotPose3d_wpiBlue(name);
    } else {
      return new Pose3d();
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
}
