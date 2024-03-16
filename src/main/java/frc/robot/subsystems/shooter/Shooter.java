package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    private ControlMode mode = ControlMode.STOP;
    private double leftVelocity, rightVelocity;

    public Shooter() {
    }


    @Override
    public void periodic() {
        switch (mode) {

          case STOP: 
            
            break;

          case VELOCITY:
            
            break;

          case DASH_VOLTAGE:

            break;

          case TEST:

            break;
        }

      }

}
