package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleAutoLogged extends SwerveModuleIO.SwerveModuleIOInputs implements LoggableInputs  {

    @Override
    public void toLog(LogTable table) {
        table.put("speedVelocity", speedVelocity);
        table.put("angleVelocity", angleVelocity);
        table.put("speedTemp", speedTemp);
        table.put("speedVoltage", speedVoltage);
        table.put("angleTemp", angleTemp);
        table.put("angleVoltage", angleVoltage);
        table.put("CAN Pose Degree", CANPosDegree);
        table.put("speedPose", speedPos);
        table.put("anglePose", anglePos);
    }

    @Override
    public void fromLog(LogTable table) {
        table.get("speedVelocity", speedVelocity);
        table.get("angleVelocity", angleVelocity);
        table.get("speedTemp", speedTemp);
        table.get("speedVoltage", speedVoltage);
        table.get("angleTemp", angleTemp);
        table.get("angleVoltage", angleVoltage);
        table.get("CAN Pose Degree", CANPosDegree);
        table.get("speedPose", speedPos);
        table.get("anglePose", anglePos);
    }
    
}
