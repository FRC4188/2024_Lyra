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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase {
  private static Shoulder instance;

  public static synchronized Shoulder getInstance() {
    if (instance == null) instance = new Shoulder();
    return instance;
  }

  // DataLog log = DataLogManager.getLog();

  // DoubleLogEntry positionLog = new DoubleLogEntry(log, "shoulder/position");
  // DoubleLogEntry voltageLog = new DoubleLogEntry(log, "shoulder/voltage");

  private CSP_TalonFX leader = new CSP_TalonFX(Constants.ids.SHOULDER_LEADER, "canivore");
  private CSP_TalonFX follower = new CSP_TalonFX(Constants.ids.SHOULDER_FOLLOWER, "canivore");
  private CSP_CANcoder encoder = new CSP_CANcoder(Constants.ids.SHOULDER_ENCODER, "canivore");

  private ProfiledPIDController pid = Constants.shoulder.SHOULDER_PID;

  private ArmFeedforward ff = Constants.shoulder.ARM_FEEDFORWARD;

  private double angle = -32.5;

  /** Creates a new Shoulder. */
  public Shoulder() {
    init();
  }

  @Override
  public void periodic() {
    // positionLog.append(getAngle().getDegrees());
    // voltageLog.append(getVoltage());

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder Angle", getAngle().getDegrees());
    SmartDashboard.putNumber("Shoulder Setpoint", pid.getGoal().position); 

    pid.setGoal(angle);
    setVoltage(pid.calculate(getAngle().getDegrees()) + ff.calculate(Math.toRadians(angle + 90.0), 0.0));
  }

  public void init() {

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
  }

  public void disable() {
    leader.disable();
    follower.disable();
  }

  public void refreshPID() {
    pid.reset(getAngle().getDegrees());
  }

  public double getVoltage() {
    return leader.getMotorVoltage().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    // if (getAngle().getRadians() > Constants.shoulder.UPPER_LIMIT && percent > 0.0) percent = 0.0;
    // else if (getAngle().getRadians() < Constants.shoulder.LOWER_LIMIT && percent < 0.0) percent = 0.0;
    leader.setVoltage(voltage);
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
    this.angle = clamp(angle.getDegrees(),
      Constants.shoulder.LOWER_LIMIT, Constants.shoulder.UPPER_LIMIT);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(encoder.getPositionDegrees() / Constants.shoulder.CANCODER_GEAR_RATIO);
  }

  public boolean atGoal(Rotation2d angle) {
    return Math.abs(angle.getDegrees() - getAngle().getDegrees()) < Constants.shoulder.ALLOWED_ERROR;
  }

  public boolean atGoal(Rotation2d angle, double tolerance) {
    return Math.abs(angle.getDegrees() - getAngle().getDegrees()) < tolerance;
  }
}