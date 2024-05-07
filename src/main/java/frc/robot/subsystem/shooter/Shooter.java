package frc.robot.subsystem.shooter;

import frc.robot.Robot;

public class Shooter {
    private final ShooterIO io;

    private static Shooter instance;
    public static synchronized Shooter getInstance() {
        if (instance == null) {
            if (Robot.isReal()) instance = new Shooter(new ShooterHardwareIO());
            else instance = new Shooter(new ShooterSimulationIO());
        }

        return instance;
    }
    
    private Shooter(ShooterIO io) {
        this.io = io;
    }
}
