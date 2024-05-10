package frc.robot.subsystem.shooter;

import lib.logging.IO;
import lib.logging.LoggingFunctions;
import lib.logging.values.LoggedDashboardInput;
import lib.logging.values.LoggedDashboardValue;

public abstract class ShooterIO implements IO {

    protected LoggedDashboardInput dashboardVelocity =
        new LoggedDashboardInput("Shooter/Velocity", "DashboardVelocity");
    protected LoggedDashboardValue<Double> measuredVelocity =
        new LoggedDashboardValue<Double>("Shooter/Velocity", "MeasuredVelocity", LoggingFunctions.numberLogger());
    protected LoggedDashboardValue<Double> setVelocity =
        new LoggedDashboardValue<Double>("Shooter/Velocity", "SetVelocity", LoggingFunctions.numberLogger());
    
    protected LoggedDashboardValue<Double> measuredAmps =
        new LoggedDashboardValue<>("Shooter/Amps", "MeasuredAmps", LoggingFunctions.numberLogger());
    
    protected LoggedDashboardValue<Double> measuredTemp =
        new LoggedDashboardValue<>("Shooter/Temp", "MeasuredTemp", LoggingFunctions.numberLogger());
    
    protected LoggedDashboardInput dashboardVoltage =
        new LoggedDashboardInput("Shooter/Voltage", "DashboardVoltage");
    protected LoggedDashboardValue<Double> measuredVoltage =
        new LoggedDashboardValue<Double>("Shooter/Voltage", "MeasuredVoltage", LoggingFunctions.numberLogger());
    protected LoggedDashboardValue<Double> setVoltage =
        new LoggedDashboardValue<Double>("Shooter/Voltage", "SetVoltage", LoggingFunctions.numberLogger());

    public abstract void setVelocity(double velocity);
    public abstract double getVelocity();

    public abstract void setVoltage(double volts);
    public abstract double getVoltage();

    public abstract double getAmp();

    public abstract double getTemp();

    public abstract void setBrake(boolean braking);
}
