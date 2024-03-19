package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class ShotConstants {


    public static enum BlindShots {
        SPEAKER_DIRECT(13.0, 32.5),
        SPEAKER_DEFENDED(12.0, 45.0);

        private final double velocity;
        private final double angle;
        private BlindShots(double velocity, double angle) {
            this.velocity = velocity;
            this.angle = angle;
        }

        public double getVelocity() {
            return velocity;
        }

        public double getAngle() {
            return angle;
        }
    }   

    public static class DataPoints{
        public final double distance;
        public final double value;
  
        public DataPoints(double distance, double value){
          this.distance = distance;
          this.value = value;
        }
    }
  
    public static final DataPoints[] VELOCITY_DATA_POINTS = {
        new DataPoints(0, 0),
        new DataPoints(0, 0),
        new DataPoints(0, 0)
    };
    public static final DataPoints[] ANGLE_DATA_POINTS = {
        new DataPoints(0, 0),
        new DataPoints(0, 0),
        new DataPoints(0, 0)
    };

    public static Translation3d CURRENT_SPEAKER_LOCATION = 
        DriverStation.getAlliance().get() == Alliance.Blue ?  
        Constants.field.BLUE_SPEAKER_LOCATION :
        Constants.field.RED_SPEAKER_LOCATION;


    public static InterpolatingDoubleTreeMap VELOCITY_SPEAKER = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap ANGLE_SPEAKER = new InterpolatingDoubleTreeMap();

    public enum Goal {
      SPEAKER(
        ANGLE_SPEAKER, VELOCITY_SPEAKER,
        new Pose2d(CURRENT_SPEAKER_LOCATION.getX(), CURRENT_SPEAKER_LOCATION.getY(), Rotation2d.fromRadians(Math.PI)), // TODO: change rotation based on color
        10, //TODO: happy zone tuning
        Units.inchesToMeters(3.0 * 12.0 + 5.0 + (3.0 / 8.0)));


  

      public final InterpolatingDoubleTreeMap ITM_A, ITM_V;
      public final Pose2d position;
      public final double happyZone;
      public final double goalWidth;

      private Goal(InterpolatingDoubleTreeMap ITM_A, InterpolatingDoubleTreeMap ITM_V, Pose2d position, double happyZone, double goalWidth){
        this.ITM_A = ITM_A;
        this.ITM_V = ITM_V;
        this.position = position;
        this.happyZone = happyZone;
        this.goalWidth = goalWidth;
      }

    }
}