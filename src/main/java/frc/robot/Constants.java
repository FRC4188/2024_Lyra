// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class controller {
    public static final int PILOT_PORT = 0;
    public static final int COPILOT_PORT = 1;
    public static final double DEADBAND = 0.15;
    public static final double TRIGGER_THRESHOLD = 0.6;
  }

  public static final class field {
    public static Translation2d IMPORTANT_GOAL = new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19));

    public static double FIELD_WIDTH = Units.feetToMeters(26.2916);

    public static Transform3d RED_RIGHT_WALL =
        new Transform3d(
            new Translation3d(FIELD_WIDTH, new Rotation3d(0, 0, Math.PI)),
            new Rotation3d(0, 0, Math.PI));
  }

  public static final class robot {
    public static final double A_LENGTH = Units.inchesToMeters(24.0); // Axel length (Meters).
    public static final double A_WIDTH = Units.inchesToMeters(24.0); // Axel width (Meters).
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double FALCON_ENCODER_TICKS =
        2048.0; // Counts per revolution of the Falcon 500 motor.
    public static final double FALCON_MAX_VEL = 6380.0;

    public static final double MAX_TEMP = 50.0;
  }

  public static final class ids {
    
    public static final int FR_SPEED = 1;
    public static final int FR_ANGLE = 2;
    public static final int FR_ENCODER = 11;

    public static final int FL_SPEED = 3;
    public static final int FL_ANGLE = 4;
    public static final int FL_ENCODER = 12;

    public static final int BL_SPEED = 5;
    public static final int BL_ANGLE = 6;
    public static final int BL_ENCODER = 13;

    public static final int BR_SPEED = 7;
    public static final int BR_ANGLE = 8;
    public static final int BR_ENCODER = 14;

    public static final int PIGEON = 15;

    public static final int INTAKE = 16;

    public static final int SHOULDER_LEADER = 17;
    public static final int SHOULDER_FOLLOWER = 18;

    public static final int SHOULDER_ENCODER = 23;

    public static final int LEFT_SHOOTER = 19;
    public static final int RIGHT_SHOOTER = 20;

    public static final int LEFT_CLIMBER = 21;
    public static final int RIGHT_CLIMBER = 22;
  }

  public static class drivetrain {
    public static final double DRIVE_GEARING = 5.14; // 6.55 : 1
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_TICKS_PER_ROTATION =
        robot.FALCON_ENCODER_TICKS * DRIVE_GEARING;
    public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;
    public static final double DRIVE_TICKS_PER_METER =
        DRIVE_GEARING / WHEEL_CIRCUMFRENCE;
    public static final double DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;

    public static final double ANGLE_GEARING = 11.3142; // 10.29 : 1
    public static final double ANGLE_TICKS_PER_ROTATION =
        robot.FALCON_ENCODER_TICKS * ANGLE_GEARING;
    public static final double ANGLE_TICKS_PER_DEGREE = ANGLE_TICKS_PER_ROTATION / 360.0;
    public static final double ANGLE_DEGREES_PER_TICK = 1.0 / ANGLE_TICKS_PER_DEGREE;

    public static final double MAX_VOLTS = 12.0;
    public static final double MAX_VELOCITY = 10.0; // ?
    public static final double MAX_ACCEL = 7.0;
    public static final double MAX_CACCEL = 8.0;
    public static final double MAX_RADIANS = 5 * Math.PI;
    public static final double RAMP_RATE = 0.5;

    public static final Matrix<N3, N1> STATE_STD_DEVS =
        VecBuilder.fill(0.1, 0.1, 0.01); // [x, y, theta]
    public static final Matrix<N3, N1> VISION_STD_DEVS =
        VecBuilder.fill(1.0, 1.0, 100.0); // [x, y, theta]

    public static final Translation2d FL_LOCATION =
        new Translation2d((Constants.robot.A_LENGTH / 2), (Constants.robot.A_WIDTH / 2));
    public static final Translation2d FR_LOCATION =
        new Translation2d((Constants.robot.A_LENGTH / 2), -(Constants.robot.A_WIDTH / 2));
    public static final Translation2d BL_LOCATION =
        new Translation2d(-(Constants.robot.A_LENGTH / 2), (Constants.robot.A_WIDTH / 2));
    public static final Translation2d BR_LOCATION =
        new Translation2d(-(Constants.robot.A_LENGTH / 2), -(Constants.robot.A_WIDTH / 2));

    public static final double FL_ZERO = 145.54687500000003;
    public static final double BL_ZERO = 117.59765625;
    public static final double BR_ZERO = -146.42578125;
    public static final double FR_ZERO = 88.330078125;

  public static final PIDController ANGLE_PID = new PIDController(0.008, 0.0, 0.0);
    public static final SimpleMotorFeedforward ANGLE_FF = new SimpleMotorFeedforward(0.0, 1);

    public static final PIDController SPEED_PID = new PIDController(0.1, 0.0, 0.0);
    public static final SimpleMotorFeedforward SPEED_FF = new SimpleMotorFeedforward(0, 0);

    // public static final PIDConstants XY_PID = new PIDConstants(3.0, 0.0, 0.0);

    // public static final PIDConstants ROT_PID = new PIDConstants(0.0, 0.0, 0.0);

    // public static final PIDConstants CORRECTION_PID = new PIDConstants(-0.1, 0.0, -0.006);

    public static final class correctionPID {
      public static final double kP = 0.1;
      public static final double kI = 0.0;
      public static final double kD = 0.00;
    }

  }

  public static final class shoulder {

    public static final double MAX_VEL = 0.0;
    public static final double MAX_ACCEL = 0.0;

    public static final PIDConstants SHOULDER_PID = new PIDConstants(0, 0, 0);
    public static final Constraints CONSTRAINTS = new Constraints(MAX_VEL, MAX_ACCEL);
    public static final ArmFeedforward ARM_FEEDFORWARD = new ArmFeedforward(0, 0, 0);

    public static final double ZERO = 0;
    public static final double ALLOWED_ERROR = 0;
  }


  public static final class sensors {
    public static final class pigeon {
      public static final double PIGEON_OFFSET_DEGREES = 0.0;
    }

    public static final class limelight {
      public static final String LEFT_NAME = "limelight-left";
      public static final String RIGHT_NAME = "limelight-right";
      
      public static final Translation3d LEFT_POSITION = new Translation3d(0, 0, 0);
      public static final Translation3d RIGHT_POSITION = new Translation3d(0, 0, 0);

      public static final Rotation3d LEFT_ROTATION = new Rotation3d(0, 0, 0);
      public static final Rotation3d RIGHT_ROTATION = new Rotation3d(0, 0, 0);
    }
    




  }
}
