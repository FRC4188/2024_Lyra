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

  public CSP_TalonFX leftClimber = new CSP_TalonFX(Constants.ids.CLIMBER_LEFT);
  public CSP_TalonFX rightClimber = new CSP_TalonFX(Constants.ids.CLIMBER_RIGHT);

  public DigitalInput leftLimit = new DigitalInput(Constants.ids.CLIMBER_LEFT_LIMIT);
  public DigitalInput rightLimit = new DigitalInput(Constants.ids.CLIMBER_RIGHT_LIMIT);

  public ProfiledPIDController leftPID = new ProfiledPIDController(Constants.climber.LEFT_kP, Constants.climber.LEFT_kP, Constants.climber.LEFT_kP, new Constraints(Constants.climber.MAX_VELOCITY, Constants.climber.MAX_ACCEL));
  public ProfiledPIDController rightPID = new ProfiledPIDController(Constants.climber.RIGHT_kP, Constants.climber.RIGHT_kP, Constants.climber.RIGHT_kP, new Constraints(Constants.climber.MAX_VELOCITY, Constants.climber.MAX_ACCEL));
 
  public ArmFeedforward leftFF = new ArmFeedforward(Constants.climber.LEFT_kS, Constants.climber.LEFT_kG, Constants.climber.LEFT_kV);
  public ArmFeedforward rightFF = new ArmFeedforward(Constants.climber.RIGHT_kS, Constants.climber.RIGHT_kG, Constants.climber.RIGHT_kV);

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

  /**
   * Sets the right climber motor to a given power
   * @param power from 0 to 1, the amount of power set to the motor
   */

  public void setRight(double power) {
    rightClimber.set(power / RobotController.getBatteryVoltage());
  }

  /**
   * Sets the position of both climber motors to a given position using PID and Feedforward
   * @param position a set position that the motor can go to
   */

  public void setPosition(double position) {
    setLeft(leftPID.calculate(getPositionLeft(), position) + leftFF.calculate(getPositionLeft(), position));
    setRight(rightPID.calculate(getPositionRight(), position) + rightFF.calculate(getPositionRight(), position));
  }

  /**
   * Returns the position of the left climber motor
   * @return leftClimber current rotor position as a double
   */

  public double getPositionLeft() {
    return leftClimber.getRotorPosition().getValueAsDouble() * Constants.climber.METERS_PER_ROT_LEFT;
  }

  /**
   * Returns the position of the right climber motor
   * @return rightClimber current rotor position as a double
   */

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

  /**
   * Gives the velocity of the right climber motor, in meters per second
   * @return double velocity
   */

  public double getVelocityRight() {
    return rightClimber.getRPM() / 60.0 * Constants.climber.METERS_PER_ROT_RIGHT;
  }

  /**
   * Resets the position of the left climber motor, to a certain position
   * @param position double reset position
   */

  public void resetPositionLeft(double position) {
    leftClimber.setEncoderDegrees(position * Constants.climber.METERS_PER_ROT_LEFT);
  }

  /**
   * Resets the position of the right climber motor, to a certain position
   * @param position double reset position
   */

  public void resetPositionRight(double position) {
    leftClimber.setEncoderDegrees(position * Constants.climber.METERS_PER_ROT_RIGHT);
  }

  /**
   * Gives the temperature of the left climber motor
   * @return temperature in Celsius
   */

  public double getLeftTemp() {
    return leftClimber.getTemperature();
  }

  /**
   * Gives the temperature of the right climber motor
   * @return temperature in Celsius
   */

  public double getRightTemp() {
    return rightClimber.getTemperature();
  }
}