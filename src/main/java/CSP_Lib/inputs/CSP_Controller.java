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
  private short m_leftRumble;
  private short m_rightRumble;

  private final double DEFAULT_DEADBAND = 0.1;
  private double stickDeadband = DEFAULT_DEADBAND;
  private double triggerDeadband = DEFAULT_DEADBAND;

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
  private Translation2d getOutput(Translation2d input, Scale scale) {
    double norm = input.getNorm();

    if (norm > stickDeadband) {
      switch (scale) {
        case CUBED:
          norm = norm * norm * norm;
          break;
        case QUARTIC:
          norm = norm * norm * norm * norm;
          break;
         case SQUARED:
         norm = norm * norm;
        default:
          break;
      }

      return new Translation2d((norm - stickDeadband) / (1 - stickDeadband), input.getAngle());
    }
    
    return new Translation2d();
  }

  private double getOutput(double input, Scale scale) {
    if (input > stickDeadband) {
      switch (scale) {
        case CUBED:
          input = input * input * input;
          break;
        case QUARTIC:
          input = Math.signum(input) * input * input * input * input;
          break;
        case SQUARED:
         input = Math.signum(input) * input * input;
        default:
          break;
      }

      return (input - triggerDeadband) / (1 - triggerDeadband);
    }
    
    return 0.0;
  }

  public Translation2d getRightStick() {
    return getRightStick(Scale.LINEAR);
  }

  public Translation2d getRightStick(Scale scale) {
    return getOutput(new Translation2d(super.getRightX(), super.getRightY()), scale);
  }

  public Translation2d getLeftStick() {
    return getLeftStick(Scale.LINEAR);
  }

  public Translation2d getLeftStick(Scale scale) {
    return getOutput(new Translation2d(super.getLeftX(), super.getLeftY()), scale);
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
    return this.rightTrigger();
  }

  public Trigger getLeftTButton() {
    return this.leftTrigger();
  }

  public void setRumble(RumbleType type, double value) {
    getHID().setRumble(type, value);
  }

  public void setStickDeadband(double deadband) {
    this.stickDeadband = deadband;
  }

  public void setTriggerDeadband(double deadband) {
    this.triggerDeadband = deadband;
  }
}
