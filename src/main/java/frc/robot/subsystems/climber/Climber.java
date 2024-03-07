package frc.robot.subsystems.climber;

import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

  

  public CSP_TalonFX leftClimber = new CSP_TalonFX(Constants.ids.LEFT_CLIMBER);
  public CSP_TalonFX rightClimber = new CSP_TalonFX(Constants.ids.RIGHT_CLIMBER);

  public DigitalInput leftLimit = new DigitalInput(Constants.ids.CLIMBER_LEFT_LIMIT);
  public DigitalInput rightLimit = new DigitalInput(Constants.ids.CLIMBER_RIGHT_LIMIT);

  public ProfiledPIDController leftPID = new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0, 0.0));
  public ProfiledPIDController rightPID = new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0, 0.0));
 
  public ElevatorFeedforward leftFF = new ElevatorFeedforward(0.0, 0.0, 0.0);
  public ElevatorFeedforward rightFF = new ElevatorFeedforward(0.0, 0.0, 0.0);

  private Notifier notifier = new Notifier(() -> {
    updateDashboard();
  });

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));


    private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                leftClimber.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("climber-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            leftClimber.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getPositionLeft(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getVelocityLeft(), MetersPerSecond));
               
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  public Climber() {

    leftClimber.setEncoderDegrees(0.0);
    rightClimber.setEncoderDegrees(0.0);

    leftClimber.setInverted(false);
    rightClimber.setInverted(true);

    leftClimber.setBrake(true);
    rightClimber.setBrake(true);

    notifier.startPeriodic(0.2);
  }

  /**
   * Updates SmartDashboard for the climber
   */

  public void updateDashboard() {
    SmartDashboard.putNumber("Left Climber", getPositionLeft());
    SmartDashboard.putNumber("Right Climber:", getPositionRight());

    SmartDashboard.putNumber("LClimber Temp:", getLeftTemp());
    SmartDashboard.putNumber("RClimber Temp:", getRightTemp());
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
   * @param position a set position that the motor can go to (in meters)
   */

  public void setPosition(double position) {
    setLeft(leftPID.calculate(getPositionLeft(), position / Constants.climber.METERS_PER_ROT_LEFT) + leftFF.calculate(getPositionLeft(), position / Constants.climber.METERS_PER_ROT_LEFT));
    setRight(rightPID.calculate(getPositionRight(), position / Constants.climber.METERS_PER_ROT_RIGHT) + rightFF.calculate(getPositionRight(), position / Constants.climber.METERS_PER_ROT_RIGHT));
  }

  /**
   * Sets the position of both climber motors to a given position using PID and Feedforward
   * @param position a set position that the motor can go to (in rotations)
   */

  public void setPositionRotations(double position) {
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

  public void setLeftPID(double p, double i, double d) {
    leftPID.setPID(p, i, d);
  }

  public void setRightPID(double p, double i, double d) {
    rightPID.setPID(p, i, d);
  }

  public void setLeftP(double p) {
    leftPID.setP(p);
  }

  public void setLeftI(double i) {
    leftPID.setI(i);
  }

  public void setLeftD(double d) {
    leftPID.setD(d);
  }

  public void setRightP(double p) {
    rightPID.setP(p);
  }

  public void setRightI(double i) {
    rightPID.setI(i);
  }

  public void setRightD(double d) {
    rightPID.setD(d);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}