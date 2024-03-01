package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static Shooter instance = null;
    private Flywheel flywheel = Flywheel.getInstance();

    public static synchronized Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }

    public boolean atRPM(double RPM) {
        return false;
    }

   
}