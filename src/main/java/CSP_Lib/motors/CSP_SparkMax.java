package CSP_Lib.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class CSP_SparkMax extends CANSparkMax implements CSP_Motor {
  private RelativeEncoder encoder;

  public CSP_SparkMax(int id) {
    super(id, MotorType.kBrushless);
    encoder = getEncoder();
    init();
  }

  public void init() {
    super.restoreFactoryDefaults();
    super.clearFaults();
    setEncoder(0);
  }

  public void set(double percent) {
    super.set(percent);
  }

  public void setVoltage(double voltage) {
    super.setVoltage(voltage);
  }

  public void setRampRate(double rate) {
    super.setOpenLoopRampRate(rate);
    super.setClosedLoopRampRate(rate);
  }

  public void setInverted(boolean inverted) {
    super.setInverted(inverted);
  }

  public void setBrake(boolean brake) {
    super.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setEncoder(double position) {
    encoder.setPosition(position / (2.0 * Math.PI));
  }

  public double getRPM() {
    return encoder.getVelocity();
  }

  public double getPositionRads() {
    return encoder.getPosition() * 2 * Math.PI;
  }

  public double getTemperature() {
    return super.getMotorTemperature();
  }

  public double getCurrent() {
    return super.getOutputCurrent();
  }

  public int getID() {
    return super.getDeviceId();
  }
}
