package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import CSP_Lib.motors.CSP_CANcoder;
import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveModuleIO.SwerveModuleIOInputs;

public class SwerveModule {

  private final Constants.drivetrain.SwerveModuleConfig config;
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputs inputs;

  private final PIDController anglePID;
  private final SimpleMotorFeedforward angleFF;
  private final PIDController speedPID;
  private final SimpleMotorFeedforward speedFF;

  public SwerveModule(Constants.drivetrain.SwerveModuleConfig config) {
    this.config = config;
    this.io = switch(Constants.robot.getRobotMode()){
      case REAL -> new SwerveModuleIOReal(config);
      case SIM -> new SwerveModuleIOSim(config);
    };
    this.inputs = new SwerveModuleIOInputs();

    this.anglePID = Constants.drivetrain.ANGLE_PID;
    this.angleFF = Constants.drivetrain.ANGLE_FF;
    this.speedPID = Constants.drivetrain.SPEED_PID;
    
    this.speedFF = new SimpleMotorFeedforward(Constants.drivetrain.SPEED_FF.ks, Constants.drivetrain.SPEED_FF.kv);

    // TempManager.addMotor(this.speed);
    // TempManager.addMotor(this.angle);

    io.config();
  }

  public void updateInputs(){}

  //TODO: check if works
  public void setModuleState(SwerveModuleState desired) {
    SwerveModuleState optimized =
        SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(getAngleDegrees()));

    double velocity = optimized.speedMetersPerSecond / Constants.drivetrain.DRIVE_METERS_PER_TICK;
    // pseudocode : setVolts(PID + FF)

    double speedVolt = speedPID.calculate(getVelocity(), velocity) + speedFF.calculate(velocity);
    // angle.setVoltage(angleFF.calculate(anglePID.calculate(getAngleDegrees(), optimized.angle.getDegrees())));
    double angleVolt = (anglePID.calculate(getAngleDegrees(), optimized.angle.getDegrees()));

    io.setVoltage(speedVolt, angleVolt);
  }

  /** Sets the speed and angle motors to zero power */
  public void zeroPower() {
    io.setVoltage(0.0, 0.0);
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

  /**
   * Gives the velocity of the angle
   * 
   * @return double with velocity in Meters Per Second
   */
  private double getVelocity() { // RPM to MPS
    return (inputs.speedVelocity * Constants.drivetrain.WHEEL_CIRCUMFRENCE) / config.gearRatio();
  }

  /**
   * Gives the angle of the module and constantly corrects it with Cancoder reading
   *
   * @return the angle with range [-180 to 180]
   */
  public double getAngleDegrees() {
    return inputs.CANPosDegree; 
  }

  public double getAnglePIDSetpoint() {
    return anglePID.getSetpoint();
  }

  private double getPositionMeters() {
    return ((inputs.speedPos * 360.0) * Constants.drivetrain.WHEEL_CIRCUMFRENCE) / (360.0 * config.gearRatio());
  }

  public String getName() {
    return config.name();
  }

  public Translation2d getLocation() {
    return config.location();
  }

  public void setAnglePIDConstants(double kP, double kI, double kD) {
    anglePID.setPID(kP, kI, kD);
  }

  public void setSpeedPIDConstants(double kP, double kI,double kD) {
    speedPID.setPID(kP, kI, kD);
  }

  public double getPositionDegrees() {
    return inputs.speedPos * 360.0;
  }

  public double getAngleTemp() {
    return inputs.angleTemp;
  }

  public double getAngleVolts() {
    return inputs.angleVoltage;
  }

}
