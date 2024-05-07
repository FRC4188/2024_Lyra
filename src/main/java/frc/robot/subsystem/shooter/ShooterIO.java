package frc.robot.subsystem.shooter;

import lib.logging.IO;
import lib.logging.LoggingFunctions;
import lib.logging.values.LoggedDashboardInput;
import lib.logging.values.LoggedDashboardValue;

public abstract class ShooterIO implements IO {

    private LoggedDashboardInput dashboardVelocity =
        new LoggedDashboardInput("Shooter/Velocity", "DashboardVelocity");
    private LoggedDashboardValue<Double> measuredVelocity =
        new LoggedDashboardValue<Double>("Shooter/Velocity", "MeasuredVelocity", LoggingFunctions.numberLogger());
    private LoggedDashboardValue<Double> setVelocity =
        new LoggedDashboardValue<Double>("Shooter/Velocity", "SetVelocity", LoggingFunctions.numberLogger());
    
    private LoggedDashboardValue<Double> measuredAmps =
        new LoggedDashboardValue<>("Shooter/Amps", "MeasuredAmps", LoggingFunctions.numberLogger());
    
    private LoggedDashboardValue<Double> measuredTemp =
        new LoggedDashboardValue<>("Shooter/Temp", "MeasuredTemp", LoggingFunctions.numberLogger());
    
    private LoggedDashboardInput dashboardVoltage =
        new LoggedDashboardInput("Shooter/Voltage", "DashboardVoltage");
    private LoggedDashboardValue<Double> measuredVoltage =
        new LoggedDashboardValue<Double>("Shooter/Voltage", "MeasuredVoltage", LoggingFunctions.numberLogger());
    private LoggedDashboardValue<Double> setVoltage =
        new LoggedDashboardValue<Double>("Shooter/Voltage", "SetVoltage", LoggingFunctions.numberLogger());

    public abstract void setVelocity();
    public abstract double getVelocity();

    public abstract void setVoltage();
    public abstract double getVoltage();

    public abstract double getAmp();

    public abstract double getTemp();
}
