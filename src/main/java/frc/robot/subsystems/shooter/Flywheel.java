// package frc.robot.subsystems.shooter;

// import CSP_Lib.motors.CSP_TalonFX;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.units.Distance;
// import edu.wpi.first.units.Measure;
// import edu.wpi.first.units.MutableMeasure;
// import edu.wpi.first.units.Velocity;
// import edu.wpi.first.units.Voltage;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.Constants;

// import static edu.wpi.first.units.MutableMeasure.mutable;
// import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.MetersPerSecond;
// import static edu.wpi.first.units.Units.Volts;

// public class Flywheel extends SubsystemBase{
//     private static Flywheel instance = null;
//     private CSP_TalonFX left = new CSP_TalonFX(35);
//     private CSP_TalonFX right = new CSP_Talon()

//     private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(-1.7846, 0.45975, 1.2793);
//     private PIDController pid = new PIDController(0.39011, 0, 0);

//     private double velocity = 0.0;

//     public static synchronized Flywheel getInstance() {
//       if (instance == null) instance = new Flywheel();
//       return instance;
//     }
//   // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
//   private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
//   // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
//   private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
//   // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
//   private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));


//     private final SysIdRoutine m_sysIdRoutine =
//       new SysIdRoutine(
//           // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
//           new SysIdRoutine.Config(),
//           new SysIdRoutine.Mechanism(
//               // Tell SysId how to plumb the driving voltage to the motors.
//               (Measure<Voltage> volts) -> {
//                 motor.setVoltage(volts.in(Volts));
//               },
//               // Tell SysId how to record a frame of data for each motor on the mechanism being
//               // characterized.
//               log -> {
//                 // Record a frame for the left motors.  Since these share an encoder, we consider
//                 // the entire group to be one motor.
//                 log.motor("flywheel-left")
//                     .voltage(
//                         m_appliedVoltage.mut_replace(
//                             motor.get() * RobotController.getBatteryVoltage(), Volts))
//                     .linearPosition(m_distance.mut_replace(getPosition(), Meters))
//                     .linearVelocity(
//                         m_velocity.mut_replace(getVelocity(), MetersPerSecond));
               
//               },
//               // Tell SysId to make generated commands require this subsystem, suffix test state in
//               // WPILog with this subsystem's name ("drive")
//               this));

//     public Flywheel() {
//       SmartDashboard.putNumber("Flywheel Velocity Set", 0);

//     }

//     public void updateDashboard() {
//       SmartDashboard.putNumber("Shooter Velocity", getVelocity());
//       SmartDashboard.putNumber("Shooter Voltage", getVoltage());
//       SmartDashboard.putNumber("Shooter Temperature", getTemperature());
//     }

//     @Override
//     public void periodic() {
//         setVoltage(pid.calculate(getVelocity(), velocity) + ff.calculate(velocity));
//       }

//     public void set(double percentage) {
//         motor.set(percentage);
//     }

//     public void setVelocity(double velocity) {
//         this.velocity = velocity;
//       }

//     /**
//      * Sets the voltage of the flywheel
//      * @param voltage the number of volts
//      */
//     public void setVoltage(double voltage) {
//         motor.setVoltage(voltage / RobotController.getBatteryVoltage());
//     }

//     public void disable() {
//       motor.disable();
//     }

//     /**
//      * Returns the velocity of the flywheel, in Meters Per Second
//      */
//     public double getVelocity() {
//         return (motor.getRPM() / 60) * Constants.shooter.SHOOTER_DIAMETER_METERS * Math.PI;
//     }

//     public double getVoltage() {
//         return motor.getMotorVoltage().getValueAsDouble();
//     }

//     /** 
//      * Returns the position of the flywheel, in meters 
//      */
//     public double getPosition() {
//       return motor.getPositionDegrees() / (360.0) * Constants.shooter.SHOOTER_DIAMETER_METERS * Math.PI;
//     }

//     public double getTemperature() {
//       return motor.getTemperature();
//     }


//     public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
//       return m_sysIdRoutine.quasistatic(direction);
//     }

//     public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//       return m_sysIdRoutine.dynamic(direction);
//     }

    
  
// }
