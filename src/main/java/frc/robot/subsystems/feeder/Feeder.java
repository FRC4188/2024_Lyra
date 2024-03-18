package frc.robot.subsystems.feeder;

import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
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

  private CSP_TalonFX motor = new CSP_TalonFX(Constants.ids.FEEDER, "canivore");
  private DigitalInput breaker = new DigitalInput(Constants.ids.FEEDER_BEAM_BREAKER);

  public enum ControlMode {
    STOP,
    PERCENT,
    DASH_VOLTAGE
  }
  private ControlMode controlMode = ControlMode.STOP;

  private double percent = 0;

  public Feeder() {
    motor.setBrake(true);
    motor.setInverted(false);
  }

  @Override
  public void periodic() {

      switch (controlMode) {
        case STOP:
          disable();
          break;
        
        case PERCENT:
          motor.set(percent);
          break;
        
        case DASH_VOLTAGE:

          break;
      }
  }

  public void setPercent(double percent) {
    this.percent = percent;
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

  public boolean isBroken() {
    return !breaker.get();
    // return false;
  }

  public void setControlMode(ControlMode mode) {
    this.controlMode = mode;
  }
}

