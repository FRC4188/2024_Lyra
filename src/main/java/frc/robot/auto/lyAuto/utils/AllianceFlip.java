package frc.robot.auto.lyAuto.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.sensors.Sensors;

public class AllianceFlip {

    public static double flip (double x){
        if (canFlip()) {
            return FieldConstants.fieldLength - x;
        } else {
            return x;
        }
    }

    /** Flips a translation to the correct side of the field based on the current alliance color. */
    public static Translation2d apply(Translation2d translation) {
        if (canFlip()) {
            return new Translation2d(flip(translation.getX()), translation.getY());
        } else {
            return translation;
        }
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation) {
        if (canFlip()) {
            return new Rotation2d(-rotation.getCos(), rotation.getSin());
        } else {
            return rotation;
        }
    }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
    public static Pose2d apply(Pose2d pose) {
        if (canFlip()) {
            return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
        } else {
            return pose;
        }
    }

    public static Translation3d apply(Translation3d translation3d) {
        if (canFlip()) {
            return new Translation3d(
                flip(translation3d.getX()), translation3d.getY(), translation3d.getZ());
        } else {
            return translation3d;
        }
    }

    public static boolean canFlip(){
        return Sensors.getInstance().getAllianceColor() == DriverStation.Alliance.Red;
    }
}
