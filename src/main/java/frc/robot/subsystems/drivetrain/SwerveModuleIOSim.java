package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;

import CSP_Lib.motors.CSP_CANcoder;
import CSP_Lib.motors.CSP_TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO{
    private final CSP_TalonFX speed;
    private final CSP_TalonFX angle;
    private final CSP_CANcoder encoder;

    //DCMotorSim = physic simulated motors
    private final DCMotorSim speedDCSim;
    private final DCMotorSim angleDCSim;

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

        this.angleDCSim = new DCMotorSim(
            DCMotor.getKrakenX60Foc(1),
            Constants.drivetrain.ANGLE_GEARING,
            0.0); //TODO: gotta change the values :P

        this.speedDCSim = new DCMotorSim(
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

    @Override
    public void config(){
        speed.setBrake(false);
        speed.setRampRate(Constants.drivetrain.RAMP_RATE);

        angle.setBrake(false);
        angle.setInverted(true);

        encoder.clearStickyFaults();
        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
        sensorConfigs.MagnetOffset = -(ZERO / 360.0);
        sensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        sensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoder.getConfigurator().apply(sensorConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            speedVelocity,
            angleVelocity,
            speedPos,
            anglePos,
            angleVoltage,
            speedVoltage
        );

        BaseStatusSignal.setUpdateFrequencyForAll(4.0,
         speedTemp,
         angleTemp
        );

        encoder.getAbsolutePosition().setUpdateFrequency(100.0);
        ParentDevice.optimizeBusUtilizationForAll(speed,angle,encoder);

        speed.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(250.0)
        .withSupplyCurrentLimit(50.0));
        speed.clearStickyFaults(); // clear any faults from the last time the robot was on

    
        angle.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(100.0)
        .withSupplyCurrentLimit(50.0));
        angle.clearStickyFaults();

        speed.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        angle.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    }

    @Override
    public void updateInputs(final SwerveModuleIOInputs inputs){
        BaseStatusSignal.refreshAll(
            speedVelocity,
            speedPos,
            speedTemp,
            speedVoltage,
            angleVelocity,
            angleTemp,
            angleVoltage
        );

        inputs.speedPos = speedPos.getValueAsDouble();
        inputs.speedVelocity = speedVelocity.getValueAsDouble();
        inputs.speedTemp = speedTemp.getValueAsDouble();
        inputs.speedVoltage = speedVoltage.getValueAsDouble();
        inputs.anglePos = anglePos.getValueAsDouble();
        inputs.angleVelocity = angleVelocity.getValueAsDouble();
        inputs.angleTemp = angleTemp.getValueAsDouble();
        inputs.angleVoltage = angleVoltage.getValueAsDouble();
        
    }

    @Override
    public void setInputs(final double speedVoltage, final double angleVoltage){

    };

    @Override
    public void periodic(){
        speed.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        angle.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        speed.getSimState().setRawRotorPosition(Constants.drivetrain.DRIVE_GEARING * Units.radiansToRotations(speedDCSim.getAngularPositionRad()));
        angle.getSimState().setRawRotorPosition(Constants.drivetrain.ANGLE_GEARING * Units.radiansToRotations(angleDCSim.getAngularPositionRad()));

        speed.getSimState().setRotorVelocity(Constants.drivetrain.DRIVE_GEARING * Units.radiansToRotations(speedDCSim.getAngularVelocityRadPerSec()));
        angle.getSimState().setRotorVelocity(Constants.drivetrain.ANGLE_GEARING * Units.radiansToRotations(angleDCSim.getAngularVelocityRadPerSec()));

    }

    
}
