package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    private static Feeder instance = null;

  /**
   * Singleton Constructor for {@link Shooter}
   * @return Single instance of {@link Shooter} common to all contexts.
   */
  public static synchronized Feeder getInstance() {
    if (instance == null) instance = new Feeder();
    return instance;
  }

  public Feeder() {
    
  }
}