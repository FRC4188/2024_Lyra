package frc.robot.subsystem.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;

    private static Shooter instance;
    public static synchronized Shooter getInstance() {
        if (instance == null) {
            if (Robot.isReal()) instance = new Shooter(new ShooterHardwareIO());
            else instance = new Shooter(new ShooterSimulationIO());
        }

        return instance;
    }

    private ShooterControlState state = ShooterControlState.DashboardVoltage;
    
    private Shooter(ShooterIO io) {
        this.io = io;

        io.measuredVelocity.set(0.0);
        io.measuredVoltage.set(0.0);
        io.measuredTemp.set(0.0);
        io.measuredAmps.set(0.0);
        io.setVelocity.set(0.0);
        io.setVoltage.set(0.0);
    }

    @Override
    public void periodic() {
        io.refresh();

        // System.out.print(String.format("\rrunnin: %s; volts: %s", ((double) Logger.getRealTimestamp()) * 1e-11, io.setVoltage.get()));

        io.setVoltage(io.dashboardVoltage.get());

        // switch (state) {
        //     case BrakeStop:
        //         io.setBrake(true);
        //         io.setVoltage(0.0);
        //         break;
        //     case CoastStop:
        //         io.setBrake(false);
        //         io.setVoltage(0.0);
        //         break;
        //     case DashboardVelocity:
        //         io.setVelocity(io.dashboardVelocity.get());
        //         break;
        //     case DashboardVoltage:
        //         io.setVoltage(io.dashboardVoltage.get());
        //         break;
        //     case Velocity:
        //         io.setVelocity(io.setVelocity.get());
        //         break;
        //     case Voltage:
        //         io.setVoltage(io.setVoltage.get());
        //         break;
        //     default:
        //         io.setVoltage(0.0);
        //         break;
        // }
    }
}
