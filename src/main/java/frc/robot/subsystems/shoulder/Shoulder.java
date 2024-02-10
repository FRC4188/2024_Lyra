package frc.robot.subsystems.shoulder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {

  private static Shoulder instance = null;

  /**
   * Singleton Constructor for {@link Shoulder}
   * @return Single instance of {@link Shoulder} common to all contexts.
   */
  public static synchronized Shoulder getInstance() {
    if (instance == null) instance = new Shoulder();
    return instance;
  }

  public Shoulder() {
    
  }
}