package frc.robot.subsystems.intake;

import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private static Intake instance = null;
    public static synchronized Intake getInstance() {
      if (instance == null) instance = new Intake();
      return instance;
    }

    // private DigitalInput breakerOne = new DigitalInput(Constants.ids.INTAKE_BEAM_BREAKER_1);

    private CSP_TalonFX motor = new CSP_TalonFX(Constants.ids.INTAKE, "canivore");

    private double percent = 0.0;

    public enum ControlMode {
      STOP,
      PERCENT,
      DASH_VOLTAGE
    }
    private ControlMode mode = ControlMode.STOP;

    public Intake() {
      motor.setBrake(true);
      motor.setInverted(false);
    }

    @Override
    public void periodic() {

      switch (mode) {
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
      // return !breakerOne.get();
      return false;
    }

    public void setControlMode(ControlMode mode) {
      this.mode = mode;
    }
}