package frc.robot.autoworkspace;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.lyAuto.utils.FieldConstants;
import frc.robot.commands.drivetrain.FollowPath;
import frc.robot.commands.groups.BlindReverseSpeakerShoot;
import frc.robot.commands.groups.FeedIntake;
import frc.robot.commands.groups.ShootOnReady;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Sensors;

public class testAutoCommand extends SequentialCommandGroup {

    public testAutoCommand() {
        PathPointsGen generator = PathPointsGen.getInstance();


        addCommands(
            //starting pose, need to update
            new InstantCommand(() -> Swerve.getInstance().resetOdometry(new Pose2d(15.73, 4.47, Rotation2d.fromDegrees(-180.0)))),

            new BlindReverseSpeakerShoot().withTimeout(2.0),
            new ParallelDeadlineGroup(
                new FollowPath(generator.generateTrajectory(
                    Swerve.getInstance().getPose2d().getTranslation(), 
                    FieldConstants.StagingLocations.spikeTranslations[1],
                    new TrajectoryConfig(Constants.drivetrain.MAX_VELOCITY/3, Constants.drivetrain.MAX_ACCEL/3)
                    ), Rotation2d.fromDegrees(-180.0)), 
                    
                    new FeedIntake()
            ),
                new ShootOnReady(()->0, ()->0)
            );
    }

    
}