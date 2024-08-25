package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusSignal;

import CSP_Lib.motors.CSP_CANcoder;
import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO{
    private final CSP_TalonFX speed;
    private final CSP_TalonFX angle;
    private final CSP_CANcoder encoder;

    private final DCMotorSim speedSim;
    private final DCMotorSim angleSim;

    private final double ZERO;

    private final StatusSignal<Double> speedVoltage;
    private final StatusSignal<Double> angleVoltage;
    private final StatusSignal<Double> speedTemp;
    private final StatusSignal<Double> angleTemp;
    private final StatusSignal<Double> speedVelocity;
    private final StatusSignal<Double> angleVelocity;
    private final StatusSignal<Double> speedPos;
    private final StatusSignal<Double> anglePos;

    private final double CANPosDegree;

    public SwerveModuleIOSim(Constants.drivetrain.SwerveModuleConfig config){
        this.speed = new CSP_TalonFX(config.speedId(),"rio");
        this.angle = new CSP_TalonFX(config.angleId(),"rio");
        this.encoder = new CSP_CANcoder(config.encoderId(), "rio");

        this.angleSim = new DCMotorSim(
            DCMotor.getKrakenX60Foc(1),
            Constants.drivetrain.DRIVE_GEARING,
            0.0); //TODO: gotta change the values :P
        this.speedSim = new DCMotorSim(
            DCMotor.getKrakenX60Foc(1),
            Constants.drivetrain.DRIVE_GEARING,
            0.0); //TODO: gotta change the values :P

        //TODO: figure out how to add encoder sim (change csp lib??)
        this.ZERO = config.zero();

        this.speedPos = speed.getRotorPosition(); //rotation
        this.speedTemp = speed.getDeviceTemp(); //celsius
        this.speedVoltage = speed.getMotorVoltage(); //V
        this.speedVelocity = speed.getVelocity(); //rps

        this.CANPosDegree = encoder.getPositionDegrees();

        this.anglePos = angle.getPosition();
        this.angleTemp = angle.getDeviceTemp();
        this.angleVoltage = angle.getMotorVoltage();
        this.angleVelocity = angle.getVelocity();

        //lwk dunno wut else to do here

    }

    
}
