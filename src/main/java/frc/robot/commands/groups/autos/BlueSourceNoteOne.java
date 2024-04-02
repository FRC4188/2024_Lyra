package frc.robot.commands.groups.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.FollowPath;
import frc.robot.commands.groups.BlindReverseSpeakerShoot;
import frc.robot.commands.groups.FeedIntake;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

public class BlueSourceNoteOne extends SequentialCommandGroup {
    Shoulder shoulder = Shoulder.getInstance();
    Shooter shooter = Shooter.getInstance();

    public BlueSourceNoteOne() {
        addCommands(
            new InstantCommand(() -> Swerve.getInstance().resetOdometry(new Pose2d(16.54 - 15.73, 4.47, Rotation2d.fromDegrees(-60.0)))),
            new InstantCommand(() -> Swerve.getInstance().resetOdometry(new Pose2d(16.54 - 15.73, 4.47, Rotation2d.fromDegrees(-60.0)))),
            new InstantCommand(() -> Swerve.getInstance().resetOdometry(new Pose2d(16.54 - 15.73, 4.47, Rotation2d.fromDegrees(-60.0)))),

            new BlindReverseSpeakerShoot().withTimeout(2.0),
            new ParallelDeadlineGroup(
                new FollowPath(TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(16.54 - 15.73, 4.47, Rotation2d.fromDegrees(-60.0)),
                        new Pose2d(16.54 - 12.82, 1.56, Rotation2d.fromDegrees(-20.0)),
                        new Pose2d(16.54 - 8.75, 1.07 - 0.25, Rotation2d.fromDegrees(0.0))
                    ), new TrajectoryConfig(Constants.drivetrain.MAX_VELOCITY / 4, Constants.drivetrain.MAX_ACCEL / 4)), Rotation2d.fromDegrees(0.0)),
                new FeedIntake()
            ),
            new ParallelDeadlineGroup(
                new FollowPath(TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(16.54 - 8.75, 1.07 - 0.25, Rotation2d.fromDegrees(180.0)),
                        new Pose2d(16.54 - 13.23, 1.82, Rotation2d.fromDegrees(180.0 - 38.4)),
                        new Pose2d(16.54 - 15.86, 3.85 + 1.35 , Rotation2d.fromDegrees(180.0 - 60.0))
                    ), new TrajectoryConfig(Constants.drivetrain.MAX_VELOCITY / 4, Constants.drivetrain.MAX_ACCEL / 4)), Rotation2d.fromDegrees(-60.0)),
                new FeedIntake()
            ),
            new BlindReverseSpeakerShoot().withTimeout(2.0)
        );
    }
}
