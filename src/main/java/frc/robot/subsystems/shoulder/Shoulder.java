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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase {
  private static Shoulder instance;

  public static synchronized Shoulder getInstance() {
    if (instance == null) instance = new Shoulder();
    return instance;
  }

  private CSP_TalonFX leader = new CSP_TalonFX(Constants.ids.SHOULDER_LEADER);
  private CSP_TalonFX follower = new CSP_TalonFX(Constants.ids.SHOULDER_FOLLOWER);
  private CSP_CANcoder encoder = new CSP_CANcoder(Constants.ids.SHOULDER_ENCODER);

  private ProfiledPIDController pid = Constants.shoulder.SHOULDER_PID;

  private ArmFeedforward ff = Constants.shoulder.ARM_FEEDFORWARD;

  /** Creates a new Shoulder. */
  public Shoulder() {
    init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder Encoder Angle", getAngle());
    SmartDashboard.putNumber("Shoulder Setpoint", pid.getSetpoint().position);
  }

  public void init() {
    MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
    sensorConfigs.MagnetOffset = -(Constants.shoulder.ZERO / 360.0);
    sensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    sensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(sensorConfigs);

    leader.setInverted(false);
    leader.setBrake(true);

    follower.setControl(new StrictFollower(leader.getDeviceID()));
    follower.setBrake(true);
    follower.setInverted(true);

    pid.reset(getAngle());
    pid.enableContinuousInput(-180, 180);
    pid.setTolerance(Constants.shoulder.ALLOWED_ERROR);
  }

  public void disable() {
    leader.disable();
  }

  public void set(double percent) {
    if (getAngle() > Constants.shoulder.UPPER_LIMIT && percent > 0.0) percent = 0.0;
      else if (getAngle() < Constants.shoulder.LOWER_LIMIT && percent < 0.0) percent = 0.0;
    leader.set(percent);
  }

  public void setPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

  public void setAngle(double angle) {
    set(pid.calculate(getAngle(), angle));
  }

  public double getAngle() {
    return encoder.getPositionDegrees() * Constants.shoulder.CANCODER_GEAR_RATIO;
  }

  public boolean atGoal(double angle) {
    return Math.abs(angle - getAngle()) < Constants.shoulder.ALLOWED_ERROR;
  }
}
