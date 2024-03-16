package frc.robot.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.subsystems.drivetrain.Swerve;

public class Localization {

    private static Localization instance = null;
    public static synchronized Localization getInstance() {
        if (instance == null) instance = new Localization();
        return instance;
    }
    
    private static final double DEFAULT_REFRESH_RATE = 0.01;
    private static final Notifier runner = new Notifier(() -> refresh());

    private static void refresh() {
    }

    public static void start() {
        start(DEFAULT_REFRESH_RATE);
    }

    public static void start(double refreshRate) {
        runner.startPeriodic(refreshRate);
    }

    public static Pose2d getPose() {
        return null;
    }
}
