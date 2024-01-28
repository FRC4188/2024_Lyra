// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

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


    public static double GRID_TOP_X = 0.4;
    public static double GRID_TOP_Z = Units.inchesToMeters(46.0);

    public static double GRID_MID_X = 0.8;
    public static double GRID_MID_Z = Units.inchesToMeters(34.0);

    public static double GRID_BOTTOM_X = 1.2;
    public static double GRID_BOTTOM_Z = Units.inchesToMeters(5.0);

    public static double DOUBLE_Z = 0.0;
    public static double SINGLE_Z = 0.0;

    public static double FIELD_WIDTH = Units.feetToMeters(26.2916);

    public static Transform3d RED_RIGHT_WALL =
        new Transform3d(
            new Translation3d(FIELD_WIDTH, new Rotation3d(0, 0, Math.PI)),
            new Rotation3d(0, 0, Math.PI));
  }

  public static final class robot {
    public static final double A_LENGTH = Units.inchesToMeters(27.5);
    public static final double A_WIDTH = Units.inchesToMeters(33);
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double FALCON_ENCODER_TICKS = 2048.0;
    public static final double NEO_ENCODER_TICKS = 42.0;

    public static final double MAX_TEMP = 50.0;
  }

  public static final class ids {
    public static final int FL_SPEED = 1;
    public static final int FL_ANGLE = 2;
    public static final int FL_ENCODER = 11;

    public static final int BL_SPEED = 3;
    public static final int BL_ANGLE = 4;
    public static final int BL_ENCODER = 12;

    public static final int BR_SPEED = 5;
    public static final int BR_ANGLE = 6;
    public static final int BR_ENCODER = 13;

    public static final int FR_SPEED = 7;
    public static final int FR_ANGLE = 8;
    public static final int FR_ENCODER = 14;

    public static final int PIGEON = 15;
  }

  public static class drivetrain {
    public static final double DRIVE_GEARING = 5.50; // 5.50 : 1
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;

    public static final double ANGLE_GEARING = 10.29; // 10.29 : 1
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

    public static final double FL_ZERO = -57.568359375;
    public static final double BL_ZERO = -147.041015625;
    public static final double BR_ZERO = 43.505859375 - 180;
    public static final double FR_ZERO = -75.498046875;

    public static final PIDConstants ANGLE_PID = new PIDConstants(0.008, 0.0, 0.0);

    public static final PIDConstants SPEED_PID = new PIDConstants(0.1, 0.0, 0.02);
    public static final double SPEED_kS = 0.00;
    public static final double SPEED_kV = 0.00;

    public static final PIDConstants XY_PID = new PIDConstants(3.0, 0.0, 0.0);

    public static final PIDConstants ROT_PID = new PIDConstants(0.0, 0.0, 0.0);

    public static final PIDConstants CORRECTION_PID = new PIDConstants(-0.1, 0.0, -0.006);
  }
  public static final class sensors {
    public static final String LEFT_NAME = "limelight-left";
    public static final String RIGHT_NAME = "limelight-right";

    public static final Translation3d LEFT_POSITION = new Translation3d(0, 0, 0);
    public static final Translation3d RIGHT_POSITION = new Translation3d(0, 0, 0);

    public static final Rotation3d LEFT_ROTATION = new Rotation3d(0, 0, 0);
    public static final Rotation3d RIGHT_ROTATION = new Rotation3d(0, 0, 0);
  }
}
