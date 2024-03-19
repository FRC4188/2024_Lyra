package frc.robot.sensors;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;

public class Localization {
    
    private static final double DEFAULT_REFRESH_RATE = 0.01;
    private static final Notifier runner = new Notifier(() -> refresh());

    private static Swerve drive = Swerve.getInstance();
    private static Pigeon pigeon = new Pigeon(
        Constants.ids.PIGEON, 
        Constants.sensors.pigeon.PIGEON_OFFSET_DEGREES);

    private static SwerveDrivePoseEstimator odometry;

    

    private static void refresh() {
        updateOdometry();
    }

    public static void start() {
        start(DEFAULT_REFRESH_RATE);

        odometry =
            new SwerveDrivePoseEstimator(
                drive.getKinematics(),
                getRotation2d(),
                drive.getSwerveModulePositions(drive.getModuleList()),
                new Pose2d()
                // ,Constants.drivetrain.STATE_STD_DEVS, 
                // Constants.drivetrain.VISION_STD_DEVS
                );
    }

    public static void start(double refreshRate) {
        runner.startPeriodic(refreshRate);
    }

    public static Pose2d getPose() {
        return new Pose2d();
    }

    public static Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(pigeon.getRotation());
    }

    public static void resetPigeon() {
        pigeon.reset();
    }

    public static void resetOdometry(Pose2d initPose) {
        Localization.resetPigeon();
        odometry.resetPosition(
            getRotation2d(),
            drive.getSwerveModulePositions(drive.getModuleList()),
            initPose);
    }

    public static void updateOdometry() {
        // Pose2d pose = getVisionPose();

        // if (!pose.equals(new Pose2d())) {
        //   odometry.addVisionMeasurement(pose, getLatency());
        // }

        odometry.update(
            getRotation2d(),
            drive.getSwerveModulePositions(drive.getModuleList()));
    }

    public static boolean atGoalAngle(double angleDegrees) {
        return (Math.abs(getPose().getRotation().getDegrees() - angleDegrees) < 3.0);
    }

    public static boolean isFlipped() {
        return getRotation2d().getCos() > 0.0;
    }
}