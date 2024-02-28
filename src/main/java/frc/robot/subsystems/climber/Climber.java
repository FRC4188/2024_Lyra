package frc.robot.subsystems.climber;

import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private static Climber instance = null;

  /**
   * Singleton Constructor for {@link Climber}
   * @return Single instance of {@link Climber} common to all contexts.
   */
  public static synchronized Climber getInstance() {
    if (instance == null) instance = new Climber();
    return instance;
  }

  CSP_TalonFX leftClimber = new CSP_TalonFX(Constants.ids.CLIMBER_LEFT);
  CSP_TalonFX rightClimber = new CSP_TalonFX(Constants.ids.CLIMBER_RIGHT);

  DigitalInput leftLimit = new DigitalInput(Constants.ids.CLIMBER_LEFT_LIMIT);
  DigitalInput rightLimit = new DigitalInput(Constants.ids.CLIMBER_RIGHT_LIMIT);

  ProfiledPIDController leftPID = new ProfiledPIDController(Constants.climber.LEFT_kP, Constants.climber.LEFT_kP, Constants.climber.LEFT_kP, new Constraints(Constants.climber.MAX_VELOCITY, Constants.climber.MAX_ACCEL));
  ProfiledPIDController rightPID = new ProfiledPIDController(Constants.climber.RIGHT_kP, Constants.climber.RIGHT_kP, Constants.climber.RIGHT_kP, new Constraints(Constants.climber.MAX_VELOCITY, Constants.climber.MAX_ACCEL));
 
  ArmFeedforward leftFF = new ArmFeedforward(0, 0, 0);
  ArmFeedforward rightFF = new ArmFeedforward(0, 0, 0);

  public Climber() {

    leftClimber.setEncoderDegrees(0.0);
    rightClimber.setEncoderDegrees(0.0);

    leftClimber.setInverted(false);
    rightClimber.setInverted(true);
  }

  /**
   * Sets the left climber motor to a given power
   * @param power from 0 to 1, the amount of power set to the motor
   */
  public void setLeft(double power) {
    leftClimber.set(power / RobotController.getBatteryVoltage());
}

  public void setRight(double power) {
    rightClimber.set(power / RobotController.getBatteryVoltage());
  }

  public void setPosition(double position) {
    setLeft(leftPID.calculate(getPositionLeft(), position) + leftFF.calculate(getPositionLeft(), position));
    setRight(rightPID.calculate(getPositionRight(), position) + rightFF.calculate(getPositionRight(), position));
  }

  public double getPositionLeft() {
    return leftClimber.getRotorPosition().getValueAsDouble() * Constants.climber.METERS_PER_ROT_LEFT;
  }

  public double getPositionRight() {
    return rightClimber.getRotorPosition().getValueAsDouble() * Constants.climber.METERS_PER_ROT_RIGHT;
  }

  /**
   * Gives the velocity of the left climber motor, in meters per second
   * @return double velocity 
   */
  public double getVelocityLeft() {
    return leftClimber.getRPM() / 60.0 * Constants.climber.METERS_PER_ROT_RIGHT;
  }

  public double getVelocityRight() {
    return rightClimber.getRPM() / 60.0 * Constants.climber.METERS_PER_ROT_RIGHT;
  }

  public void resetPositionLeft(double position) {
    leftClimber.setEncoderDegrees(position * Constants.climber.METERS_PER_ROT_LEFT);
  }

  public void resetPositionRight(double position) {
    leftClimber.setEncoderDegrees(position * Constants.climber.METERS_PER_ROT_RIGHT);
  }

  public double getLeftTemp() {
    return leftClimber.getTemperature();
  }

  public double getRightTemp() {
    return rightClimber.getTemperature();
  }


}