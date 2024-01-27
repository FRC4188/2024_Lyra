package frc.robot.subsystems.shooter;

import CSP_Lib.motors.CSP_Falcon;
import CSP_Lib.motors.CSP_Motor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase{
    private static Flywheel instance = null;
    private CSP_Falcon motor = new CSP_Falcon(0);
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0, 0);
    private PIDController pid = new PIDController(0, 0, 0);
    private double velocity = 0.0;

    public static synchronized Flywheel getInstance() {
      if (instance == null) instance = new Flywheel();
      return instance;
    }

    public Flywheel() {

    }

    @Override
    public void periodic() {
        setVoltage(pid.calculate(getVelocity(), velocity) + ff.calculate(velocity));
      }

    public void set(double percentage) {
        motor.set(percentage);
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
      }

    /**
     * Sets the voltage of the flywheel
     * @param voltage the number of volts
     */
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage / RobotController.getBatteryVoltage());
    }

    /**
     * Returns the velocity of the flywheel, in Meters Per Second
     */
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble() * Constants.shooter.SHOOTER_DIAMETER_METERS;
    }

    
  
}
