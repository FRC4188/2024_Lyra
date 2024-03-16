// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shoulder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {
  private static Shoulder instance;

  public static synchronized Shoulder getInstance() {
    if (instance == null) instance = new Shoulder();
    return instance;
  }

  public enum ControlMode {
    STOP,
    POSITION,
    DASHBOARD_POSITION,
    VOLTAGE,
    DASHBOARD_VOLTAGE;
  }

  private ControlMode mode = ControlMode.STOP;
  private double position, voltage;

  /** Creates a new Shoulder. */
  public Shoulder() {
  }

  @Override
  public void periodic() {
    switch (mode) {

      case STOP:
        
        break;
      
      case POSITION:

        break;
      
      case DASHBOARD_POSITION:

        break;
      
      case VOLTAGE:
        applyVoltage(voltage);
        break;

      default:

        break;
    }
  }

  public void stop() {
    mode = ControlMode.STOP;
  }

  public void setPosition(Rotation2d position) {
    mode = ControlMode.POSITION;
    this.position = position.getDegrees();
  }

  public void setVoltage(double volts) {
    voltage = volts;
  }

  private void applyVoltage(double volts) {

  }

}