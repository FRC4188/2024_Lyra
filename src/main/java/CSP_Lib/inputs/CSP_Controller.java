// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package CSP_Lib.inputs;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/** Add your docs here. */
public class CSP_Controller extends CommandXboxController {

  public enum Scale {
    LINEAR,
    SQUARED,
    CUBED,
    QUARTIC
  }

  public CSP_Controller(int port) {
    super(port);
  }

  /**
   * Calculates joystick output to account for scale and deadband
   *
   * @param input input value
   * @param scale input scale
   * @return adjusted value
   */
  private double getOutput(double input, Scale scale) {
    if (Math.abs(input) > Constants.controller.DEADBAND) {
      if (scale == Scale.SQUARED) return Math.signum(input) * Math.pow(input, 2);
      else if (scale == Scale.CUBED) return Math.pow(input, 3);
      else if (scale == Scale.QUARTIC) return Math.signum(input) * Math.pow(input, 4);
      else return input;
    } else {
      return 0;
    }
  }

  public Translation2d getCorrectedRight() {
    Translation2d input = new Translation2d(super.getRightY(), super.getRightX());
    if (input.getNorm() < Constants.controller.DEADBAND) {
      return new Translation2d();
    }

    return new Translation2d((input.getNorm() - Constants.controller.DEADBAND) / (1.0 - Constants.controller.DEADBAND), input.getAngle());
  }

  public Translation2d getCorrectedLeft() {
    Translation2d input = new Translation2d(super.getLeftY(), super.getLeftX());
    if (input.getNorm() < Constants.controller.DEADBAND) {
      return new Translation2d();
    }

    return new Translation2d((input.getNorm() - Constants.controller.DEADBAND) / (1.0 - Constants.controller.DEADBAND), input.getAngle()); 
  }

  /**
   * @param scale
   * @return
   */
  public double getRightY(Scale scale) {
    return -getOutput(this.getRightY(), scale);
  }

  public double getRightX(Scale scale) {
    return getOutput(this.getRightX(), scale);
  }

  public double getLeftY(Scale scale) {
    return -getOutput(this.getLeftY(), scale);
  }

  public double getLeftX(Scale scale) {
    return -getOutput(this.getLeftX(), scale);
  }

  public Trigger getLeftS() {
    return this.leftStick();
  }

  public Trigger getRightS() {
    return this.rightStick();
  }

  public Trigger getXButton() {
    return this.x();
  }

  public Trigger getYButton() {
    return this.y();
  }

  public Trigger getAButton() {
    return this.a();
  }

  public Trigger getBButton() {
    return this.b();
  }

  public Trigger getUpButton() {
    return this.povUp();
  }

  public Trigger getDownButton() {
    return this.povDown();
  }

  public Trigger getRightButton() {
    return this.povRight();
  }

  public Trigger getLeftButton() {
    return this.povLeft();
  }

  public Trigger getLeftBumperButton() {
    return this.leftBumper();
  }

  public Trigger getRightBumperButton() {
    return this.rightBumper();
  }

  public Trigger getStartButton() {
    return this.start();
  }

  public Trigger getBackButton() {
    return this.back();
  }

  public double getRightT(Scale scale) {
    return getOutput(getRightTriggerAxis(), scale);
  }

  public double getLeftT(Scale scale) {
    return getOutput(getLeftTriggerAxis(), scale);
  }

  public Trigger getRightTButton() {
    return this.rightTrigger(0.1);
  }

  public Trigger getLeftTButton() {
    return this.leftTrigger(0.1);
  }

  public void setRumble(RumbleType type, double value) {
    getHID().setRumble(type, value);
  }
}
