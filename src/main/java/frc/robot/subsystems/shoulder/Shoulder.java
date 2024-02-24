package frc.robot.subsystems.shoulder;

import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {

    private static Shoulder instance = null;
    public double angle;
    private CSP_TalonFX leader, follower;
    private MotorController shoulder; 

  /**
   * Singleton Constructor for {@link Shooter}
   * @return Single instance of {@link Shooter} common to all contexts.
   */
  public static synchronized Shoulder getInstance() {
    if (instance == null) instance = new Shoulder();
    return instance;
  }

  public Shoulder() {
    
  }
}