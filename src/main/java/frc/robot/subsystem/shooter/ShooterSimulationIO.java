package frc.robot.subsystem.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSimulationIO extends ShooterIO {

    FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getKrakenX60(2), 1.0, 1.0);

    PIDController velocityPID = new PIDController(0.0, 0.0, 0.0);

    Timer timer = new Timer();

    @Override
    public void refresh() {
        flywheelSim.setInput(setVoltage.get());

        flywheelSim.update(0.02);
        timer.reset();

        this.measuredAmps.set(flywheelSim.getCurrentDrawAmps());
        this.measuredVelocity.set(flywheelSim.getAngularVelocityRPM());
        this.measuredVoltage.set(setVoltage.get());
        this.measuredTemp.set(20.0);
    }

    @Override
    public void setVelocity(double velocity) {
        setVelocity.set(velocity);
    }

    @Override
    public double getVelocity() {
        return measuredVelocity.get();
    }

    @Override
    public void setVoltage(double volts) {
        setVoltage.set(volts);
    }

    @Override
    public double getVoltage() {
        return measuredVoltage.get();
    }

    @Override
    public double getAmp() {
        return measuredAmps.get();
    }

    @Override
    public double getTemp() {
        return measuredTemp.get();
    }

    @Override
    public void setBrake(boolean braking) {
    }
}
