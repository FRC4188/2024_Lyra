package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import CSP_Lib.motors.CSP_CANcoder;
import CSP_Lib.motors.CSP_Falcon;
import CSP_Lib.motors.CSP_Motor;
import CSP_Lib.utils.Conversions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class SwerveModule {

  private final String MODULE_NAME;
  private final Translation2d LOCATION;
  private final double GEAR_RATIO;
  private final int SPEED_ID;
  private final int ANGLE_ID;
  private final int ENCODER_ID;
  private final double ZERO;

  private final CSP_Falcon speed;
  private final CSP_Falcon angle;
  private final CSP_CANcoder encoder;

  private final PIDController anglePID;
  private final PIDController speedPID;
  private final SimpleMotorFeedforward speedFF;

  public SwerveModule(String moduleName, Translation2d location, double gearRatio, int speedID, int angleID, int encoderID, double zero) {
    this.MODULE_NAME = moduleName;
    this.LOCATION = location;
    this.GEAR_RATIO = gearRatio;
    this.SPEED_ID = speedID;
    this.ANGLE_ID = angleID;
    this.ENCODER_ID = encoderID;
    this.ZERO = zero;

    this.anglePID = new PIDController(Constants.drivetrain.angle.kP, Constants.drivetrain.angle.kI, Constants.drivetrain.angle.kD);
    this.speedPID = new PIDController(Constants.drivetrain.speed.kP, Constants.drivetrain.speed.kI, Constants.drivetrain.speed.kD);
    this.speedFF = new SimpleMotorFeedforward(Constants.drivetrain.speed.kS, Constants.drivetrain.speed.kV);

    this.speed = new CSP_Falcon(SPEED_ID);
    this.angle = new CSP_Falcon(ANGLE_ID);
    this.encoder = new CSP_CANcoder(ENCODER_ID);

    init();
  }

  public void init() {
    speed.setBrake(true);
    speed.setRampRate(Constants.drivetrain.RAMP_RATE);

    angle.setBrake(false);
    angle.setEncoder(Conversions.degreesSignedToUnsigned(getAngleDegrees()));

    anglePID.enableContinuousInput(-180, 180);
    anglePID.setTolerance(5);    
  }

  public void setModuleState(SwerveModuleState desired) {
    SwerveModuleState optimized =
        SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(getAngleDegrees()));
    speed.setVoltage(speedPID.calculate(getVelocity(), optimized.speedMetersPerSecond));
    angle.setVoltage(anglePID.calculate(getAngleDegrees(), optimized.angle.getDegrees()));
  }

  /** Sets the speed and angle motors to zero power */
  public void zeroPower() {
    angle.setVoltage(0.0);
    speed.setVoltage(0.0);
  }

  /**
   * Gives the velocity and angle of the module
   *
   * @return SwerveModuleState object containing velocity and angle
   */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getAngleDegrees()));
  }

  /**
   * Gives the position and angle of the module
   *
   * @return SwerveModulePosition object containing position and angle
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getPositionMeters(), Rotation2d.fromDegrees(getAngleDegrees()));
  }

  private double getVelocity() {
    return (speed.getRPM() *  60.0 * Constants.drivetrain.WHEEL_CIRCUMFRENCE) / Constants.drivetrain.DRIVE_GEARING;
  }

  /**
   * Gives the angle of the module and constantly corrects it with Cancoder reading
   *
   * @return the angle with range [-180 to 180]
   */
  public double getAngleDegrees() {
    return Units.radiansToDegrees(encoder.getPositionRads());
  }

  private double getPositionMeters() {
    return (speed.getPositionRads() * Constants.drivetrain.WHEEL_CIRCUMFRENCE) / (2 * Math.PI);
  }

  public String getName() {
    return this.MODULE_NAME;
  }

  public Translation2d getLocation() {
    return this.LOCATION;
  }
}
