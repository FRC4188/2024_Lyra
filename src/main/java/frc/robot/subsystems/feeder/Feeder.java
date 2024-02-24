package frc.robot.subsystems.feeder;

import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

  private static Feeder instance = null;

  /**
   * Singleton Constructor for {@link Feeder}
   * @return Single instance of {@link Feeder} common to all contexts.
   */
  public static synchronized Feeder getInstance() {
    if (instance == null) instance = new Feeder();
    return instance;
  }

  private CSP_TalonFX motor = new CSP_TalonFX(Constants.ids.FEEDER, "rio");

  public Feeder() {
    motor.setBrake(true);
    motor.setInverted(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder AMPS", motor.getCurrent());
  }

  public void set(double percent) {
      motor.set(percent);
  }

  /**
   * Sets the voltage of the Intake
   * @param voltage the number of volts
   */
  public void setVoltage(double voltage) {
      motor.setVoltage(voltage);
  }

  /**
   * Returns the velocity of the Intake, in Rotations Per Minute
   */
  public double getVelocity() {
    return motor.getRPM(); 
  }

  public void disable() {
    motor.stopMotor();
  }
}