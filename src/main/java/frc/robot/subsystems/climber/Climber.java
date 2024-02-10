package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private static Climber instance = null;

  /**
   * Singleton Constructor for {@link Climber}
   * @return Single instance of {@link Climber} common to all contexts.
   */
  public static synchronized Climber getInstance() {
    if (instance == null) instance = new Climber();
    return instance;
  }

  public Climber() {
    
  }
}