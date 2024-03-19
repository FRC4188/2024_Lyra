package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class CompetitionLimit implements Limiter {

    private SlewRateLimiter xLimiter = new SlewRateLimiter(17.5);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(17.5);

    @Override
    public ChassisSpeeds limit(ChassisSpeeds speeds) {
        return new ChassisSpeeds(
            xLimiter.calculate(speeds.vxMetersPerSecond),
            yLimiter.calculate(speeds.vyMetersPerSecond), 
            speeds.omegaRadiansPerSecond
        );
    }
    
}
