package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private static Climber instance = null;

  private enum ControlMode {
    STOP,
    VOLTAGE,
    DASH_VOLTAGE,
    POSITION,
    DASH_POSITION;
  }
  private ControlMode mode = ControlMode.STOP;

  public Climber() {
  }


  @Override
  public void periodic() {
    switch (mode) {
      case STOP:
        
        break;

      case DASH_POSITION:

        break;

      case DASH_VOLTAGE:

        break;

      case POSITION:

        break;

      case VOLTAGE:

        break;

      default:

        break;
    }
  }
}