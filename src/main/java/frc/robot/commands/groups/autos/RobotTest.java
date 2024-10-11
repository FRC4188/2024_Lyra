package frc.robot.commands.groups.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.FollowPath;
import frc.robot.subsystems.drivetrain.Swerve;

public class RobotTest extends SequentialCommandGroup{
    Swerve s = Swerve.getInstance();

    public RobotTest(){
        addCommands(
            new FollowPath(
                TrajectoryGenerator.generateTrajectory(
                    List.of(
                        s.getPose2d(),
                        new Pose2d(
                            new Translation2d(8.22, 6.60),
                            new Rotation2d()
                        )
                    ), new TrajectoryConfig(Constants.drivetrain.MAX_VELOCITY/4, Constants.drivetrain.MAX_ACCEL/4)),
                   s.getPose2d().getRotation())
        );
    }
}
