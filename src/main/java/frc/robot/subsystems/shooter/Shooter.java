package frc.robot.subsystems.shooter;

import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    private static Shooter instance = null;
    public static synchronized Shooter getInstance() {
      if (instance == null) instance = new Shooter();
      return instance;
    }

    public enum ControlMode {
      VELOCITY,
      STOP,
      DASH_VOLTAGE,
      TEST;
    }
    private ControlMode controlMode = ControlMode.STOP;
    private double leftVelocity, rightVelocity;

    public Shooter() {
      left.setInverted(true);
      right.setInverted(false);

      left.setRampRate(0.5);
      right.setRampRate(0.5);
    }

    private CSP_TalonFX left = new CSP_TalonFX(Constants.ids.LEFT_SHOOTER, "canivore");
    private CSP_TalonFX right = new CSP_TalonFX(Constants.ids.RIGHT_SHOOTER, "canivore");

    private SimpleMotorFeedforward leftff = Constants.shooter.LEFT_SHOOTER_FEEDFORWARD;
    private SimpleMotorFeedforward rightff = Constants.shooter.RIGHT_SHOOTER_FEEDFORWARD;

    private PIDController leftpid = Constants.shooter.LEFT_SHOOTER_PID;
    private PIDController rightpid = Constants.shooter.RIGHT_SHOOTER_PID;


    @Override
    public void periodic() {
      switch (controlMode) {
        case STOP: 
          setVoltage(0.0, 0.0);
          break;
        case VELOCITY:
          setVoltage(
            leftpid.calculate(getLeftVelocity(), leftVelocity) + leftff.calculate(leftVelocity),
            rightpid.calculate(getRightVelocity(), rightVelocity) + rightff.calculate(rightVelocity));
          break;
        case DASH_VOLTAGE:
          break;
        case TEST: // do nothing while testing
          break;
      }
    }

    public void setVelocity(double leftVelocity, double rightVelocity) {
      this.leftVelocity = leftVelocity;
      this.rightVelocity = rightVelocity;
    }

    /**
     * Sets the voltage of the flywheel
     * @param voltage the number of volts
     */
    public void setVoltage(double leftVoltage, double rightVoltage) {
      left.set(leftVoltage / RobotController.getBatteryVoltage());
      right.set(rightVoltage / RobotController.getBatteryVoltage());
    }

    public void setBoth(double percent){
      left.set(percent);
      right.set(percent);
    }

    /**
     * Disables the two motors lol (just like Mukil)
     */
    public void disable() {
        left.disable();
        right.disable();
    }

    /**
     * Returns the velocity of the left flywheel, in Meters Per Second
     * @return velocity as a double
     */
    public double getLeftVelocity() {
        return (left.getRPM() / 60.0) * Constants.shooter.SHOOTER_DIAMETER_METERS * Math.PI;
        // return left.getRPM();
    }
    
    /**
     * Returns the velocity of the right flywheel, in Meters Per Second
     * @return velocity as a double
     */
    public double getRightVelocity() {
        return (right.getRPM() / 60.0) * Constants.shooter.SHOOTER_DIAMETER_METERS * Math.PI;
        //return right.getRPM();
        
    }

    /**
     * Returns the voltage of the left motor ^-^ - Mukil's girlfriend
     * @return voltage as a double
     */
    public double getLeftVoltage() {
        return left.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Returns the voltage of the right motor *~* - Not Mukil's girlfriend
     * @return voltage as a double
     */
    public double getRightVoltage() {
        return right.getMotorVoltage().getValueAsDouble();
    }

    /** 
     * Returns the position of the left flywheel, in meters 
     * @return position as a double 
     */
    public double getLeftPosition() {
      return left.getPositionDegrees() / (360.0) * Constants.shooter.SHOOTER_DIAMETER_METERS * Math.PI;
    }

    /** 
     * Returns the position of the right flywheel, in meters 
     * @return position as a double 
     */
    public double getRightPosition() {
      return left.getPositionDegrees() / (360.0) * Constants.shooter.SHOOTER_DIAMETER_METERS * Math.PI;
    }

    /**
     * Returns the temperature of the left flywheel
     * @return Temperature as a double 
     */
    public double getLeftTemperature() {
      return left.getTemperature();
    }

    /**
     * Returns the temperature of the right flywheel
     * @return Temperature as a double 
     */
    public double getRightTemperature() {
      return right.getTemperature();
    }

    /**
     * Check if shooter at desired rpm
     * @param RPM
     * @return whether or not shooter reach inputted RPM 
     */
    public boolean atRPM(double RPM) {
      return (Math.abs(left.getRPM() - RPM) < 0.5 && Math.abs(right.getRPM() - RPM) < 0.5);
    }

    public void setControlMode(ControlMode mode) {
      this.controlMode = mode;
    }
}
