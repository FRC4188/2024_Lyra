package frc.robot.subsystems.intake;

import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private static Intake instance = null;

    private CSP_TalonFX motor = new CSP_TalonFX(Constants.ids.INTAKE, "canivore");

    // private DigitalInput breakerOne = new DigitalInput(Constants.ids.INTAKE_BEAM_BREAKER_1);
    // private DigitalInput breakerTwo = new DigitalInput(Constants.ids.INTAKE_BEAM_BREAKER_2);


    public static synchronized Intake getInstance() {
      if (instance == null) instance = new Intake();
      return instance;
    }

    public Intake() {
      motor.setBrake(true);
      motor.setInverted(false);
    }

    @Override
    public void periodic() {

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

    public double getCurrent() {
      return motor.getCurrent();
    }

    public void disable() {
      motor.stopMotor();
    }

    public double getTemperature() {
      return motor.getTemperature();
    }



    public boolean isBroken() {
      // return !breakerOne.get();
      return false;
    }
}