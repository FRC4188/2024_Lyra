package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Limiter {
    
    public ChassisSpeeds limit(ChassisSpeeds speeds);
}
