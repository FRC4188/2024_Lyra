package CSP_Lib.motors;

public interface CSP_Motor {

  public void setInverted(boolean inverted);

  public void init();

  public void setBrake(boolean braking);

  public void setRampRate(double rampRate);

  public void set(double percent);

  public void setVoltage(double volts);

  public void setEncoder(double position);

  public double getRPM(); 

  public double getPositionRads(); 

  public double getTemperature();

  public double getCurrent();

  public int getID();
}
