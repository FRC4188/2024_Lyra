// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import CSP_Lib.motors.CSP_CANcoder;
import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase {
  private static Shoulder instance;

  public static synchronized Shoulder getInstance() {
    if (instance == null) instance = new Shoulder();
    return instance;
  }

  private CSP_TalonFX leader = new CSP_TalonFX(Constants.ids.SHOULDER_LEADER, "canivore");
  private CSP_TalonFX follower = new CSP_TalonFX(Constants.ids.SHOULDER_FOLLOWER, "canivore");
  private CSP_CANcoder encoder = new CSP_CANcoder(Constants.ids.SHOULDER_ENCODER, "canivore");

  private ProfiledPIDController pid = Constants.shoulder.SHOULDER_PID;
  private ArmFeedforward ff = Constants.shoulder.ARM_FEEDFORWARD;


  public enum ControlMode {
    STOP,
    POSITION,
    DASHBOARD_POSITION,
    VOLTAGE,
    DASHBOARD_VOLTAGE;
  }

  private ControlMode controlMode = ControlMode.STOP;
  private Rotation2d angle = new Rotation2d();
  private double voltage = 0;

  /** Creates a new Shoulder. */
  public Shoulder() {
    MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
    sensorConfigs.MagnetOffset = -(Constants.shoulder.ZERO / 360.0) * Constants.shoulder.CANCODER_GEAR_RATIO;
    sensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    sensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(sensorConfigs);

    leader.setInverted(true);
    leader.setBrake(true);

    leader.setRampRate(0.25);

    follower.setControl(new StrictFollower(leader.getDeviceID()));
    follower.setBrake(true);
    follower.setInverted(false);

    pid.reset(getAngle().getDegrees());
    pid.enableContinuousInput(-180, 180);
    pid.setTolerance(Constants.shoulder.ALLOWED_ERROR);
  }

  @Override
  public void periodic() {
    switch (controlMode) {

      case STOP:
        disable();
        break;
      
      case POSITION:
        pid.setGoal(angle.getDegrees());
        State setpoint = pid.getSetpoint();
        setVoltage(pid.calculate(getAngle().getDegrees()) + ff.calculate(setpoint.position, setpoint.velocity));
        break;
      
      case DASHBOARD_POSITION:

        break;
      
      case VOLTAGE:
        leader.setVoltage(voltage);
        break;

      default:
        
        break;
    }
  }

  public void disable() {
    leader.disable();
  }

  public double getVoltage() {
    return leader.getMotorVoltage().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    this.voltage = voltage;
  }

  public void set(double percent) {
    leader.set(percent);
  }

  public void setPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

  public static <T extends Comparable<T>> T clamp(T val, T min, T max) {
    if (val.compareTo(min) < 0) return min;
    else if (val.compareTo(max) > 0) return max;
    else return val;
  }

  public void setAngle(Rotation2d angle) {
    angle = Rotation2d.fromDegrees(clamp(angle.getDegrees(),
      Constants.shoulder.LOWER_LIMIT, Constants.shoulder.UPPER_LIMIT));
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(encoder.getPositionDegrees() / Constants.shoulder.CANCODER_GEAR_RATIO);
  }

  public boolean atGoal(double angleDegrees) {
    return Math.abs(angleDegrees - getAngle().getDegrees()) < Constants.shoulder.ALLOWED_ERROR;
  }

  public void setControlMode(ControlMode mode) {
    this.controlMode = mode;
  }
}