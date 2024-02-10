package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private static Shooter instance = null;

  /**
   * Singleton Constructor for {@link Shooter}
   * @return Single instance of {@link Shooter} common to all contexts.
   */
  public static synchronized Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }

  public Shooter() {
    
  }
}