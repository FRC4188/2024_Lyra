package frc.robot.subsystems.intake;

import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private static Intake instance = null;
    public static synchronized Intake getInstance() {
      if (instance == null) instance = new Intake();
      return instance;
    }

    private enum ControlMode {
      STOP,
      VOLTAGE,
      DASH_VOLTAGE
    }
    private ControlMode mode = ControlMode.STOP;

    public Intake() {
    }

    @Override
    public void periodic() {

      switch (mode) {
        case STOP:

          break;
        
        case VOLTAGE:

          break;
        
        case DASH_VOLTAGE:

          break;
      }
    }
}