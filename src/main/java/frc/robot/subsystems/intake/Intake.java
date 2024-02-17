package frc.robot.subsystems.intake;

import CSP_Lib.motors.CSP_TalonFX;
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

    private CSP_TalonFX motor = new CSP_TalonFX(Constants.ids.INTAKE, "rio");

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