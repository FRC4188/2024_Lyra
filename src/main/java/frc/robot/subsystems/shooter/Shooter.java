package frc.robot.subsystems.shooter;

import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase{
    private static Shooter instance = null;
    public static synchronized Shooter getInstance() {
      if (instance == null) instance = new Shooter();
      return instance;
    }

    // DataLog log = DataLogManager.getLog();
    // DoubleLogEntry velocityLog = new DoubleLogEntry(log, "shooter/velocity");
    // DoubleLogEntry voltageLog = new DoubleLogEntry(log, "shooter/voltage");

    public enum ControlMode {
      VELOCITY, STOP, DASH_VOLTAGE, TEST;
    }

    private ControlMode controlMode = ControlMode.STOP;

    private CSP_TalonFX left = new CSP_TalonFX(Constants.ids.LEFT_SHOOTER, "canivore");
    private CSP_TalonFX right = new CSP_TalonFX(Constants.ids.RIGHT_SHOOTER, "canivore");

    private SimpleMotorFeedforward leftff = Constants.shooter.LEFT_SHOOTER_FEEDFORWARD;
    private SimpleMotorFeedforward rightff = Constants.shooter.RIGHT_SHOOTER_FEEDFORWARD;

    private PIDController leftpid = Constants.shooter.LEFT_SHOOTER_PID;
    private PIDController rightpid = Constants.shooter.RIGHT_SHOOTER_PID;

    // public

    private double leftVelocity = 0.0;
    private double rightVelocity = 0.0;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));


    private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                right.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("flywheel-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            right.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getRightPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getRightVelocity(), MetersPerSecond));
               
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

    public Shooter() {
      SmartDashboard.putNumber("Shooter kP", 0);
      SmartDashboard.putNumber("Shooter kD", 0);             
      SmartDashboard.putNumber("Shooter kS", 0);
      SmartDashboard.putNumber("Shooter kV", 0);

      left.setInverted(true);
      right.setInverted(false);

      left.setRampRate(0.1);
      right.setRampRate(0.1);
    }

    public void updateDashboard() {
      SmartDashboard.putNumber("Left Shooter Voltage", getLeftVoltage());
      SmartDashboard.putNumber("Left Shooter Temperature", getLeftTemperature());

      SmartDashboard.putNumber("Left RPM", left.getRPM());
      SmartDashboard.putNumber("Right RPM", right.getRPM());
      
      // pid.setP(SmartDashboard.getNumber("Shooter kP", 0.0));
      // pid.setD(SmartDashboard.getNumber("Shooter kD", 0.0));
      // ff = new SimpleMotorFeedforward(SmartDashboard.getNumber("Shooter kS", 0.0), SmartDashboard.getNumber("Shooter kV", 0.0), 0.0);
    }

    @Override
    public void periodic() {
        // velocityLog.append(getLeftVelocity());
        // voltageLog.append(getLeftVoltage());

        switch (controlMode) {
          case STOP: 
            setVoltage(0.0, 0.0);
            break;
          case VELOCITY:
            setVoltage(
              leftpid.calculate(getLeftVelocity(), leftVelocity) + leftff.calculate(leftVelocity),
              rightpid.calculate(getRightVelocity(), rightVelocity) + rightff.calculate(rightVelocity)
            );
            break;
          case DASH_VOLTAGE:
            setVoltage(0.0, SmartDashboard.getNumber("Flywheel voltage set", 0.0));
            break;
          case TEST: // do nothing while testing
            break;
        }

        updateDashboard();
      }

    public void set(double leftPercentage, double rightPercentage) {
        left.set(leftPercentage);
        right.set(rightPercentage);
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
      return (left.getRPM() > RPM && right.getRPM() > RPM);
    }

    public void setControlMode(ControlMode mode) {
      this.controlMode = mode;
    }


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
    }

    
  
}
