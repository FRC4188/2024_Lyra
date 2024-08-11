package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {
    //TODO: check if need more addition
     class SwerveModuleIOInputs{
        public double speedVelocity = 0.0; //rps
        public double angleVelocity = 0.0; 
        public double speedTemp = 0.0; //celsius
        public double speedVoltage = 0.0; //V
        public double angleTemp = 0.0;
        public double angleVoltage = 0.0;
        public double CANPosDegree = 0.0; //rotations
        public double speedPos = 0.0;
        public double anglePos = 0.0;
    }
    

    default void config(){}
    default void updateInputs(final SwerveModuleIOInputs inputs){}
    default void setInputs(final double speedVoltage, final double angleVoltage){};
    
}