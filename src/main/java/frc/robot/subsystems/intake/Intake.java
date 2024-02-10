package frc.robot.subsystems.intake;

import CSP_Lib.motors.CSP_Falcon;
import CSP_Lib.motors.CSP_Motor;
import CSP_Lib.motors.CSP_SparkMax;
import CSP_Lib.utils.TempManager;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private static Intake instance = null;

    private CSP_SparkMax motor = new CSP_SparkMax(Constants.ids.INTAKE);

    public static synchronized Intake getInstance() {
      if (instance == null) instance = new Intake();
      return instance;
    }

    public Intake() {
      motor.setBrake(true);
      motor.setInverted(false);
      TempManager.addMotor(motor);
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Intake AMPS", motor.getCurrent());
    }

    public void set(double percent) {
        motor.set(percent);
    }

    /**
     * Sets the voltage of the Intake
     * @param voltage the number of volts
     */
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage / RobotController.getBatteryVoltage());
    }

    /**
     * Returns the velocity of the Intake, in Meters Per Second
     */
    public double getVelocity() {
      return motor.getRPM(); // TODO: CONVERT TO METERS PER SECOND
    }

    public void disable() {
      motor.stopMotor();
    }
  
    public void intake() {
      motor.set(1.0);
    }

    public void outtake() {
      motor.set(-0.5);
    }

}