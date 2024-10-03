package frc.robot.auto.lyAuto;

import java.util.List;

import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.auto.lyAuto.task.Action;
import frc.robot.auto.lyAuto.task.ActionManager;
import frc.robot.commands.drivetrain.FollowPath;
import frc.robot.commands.groups.FeedIntake;
import frc.robot.commands.groups.ShootOnReady;
import frc.robot.subsystems.drivetrain.Swerve;

public class AutoFactory {
    private static AutoFactory instance;
    private ActionManager actionManager;

    public static AutoFactory getInstance(){
        if(instance == null) instance = new AutoFactory();
        return instance;
    }

    private AutoFactory() {
        // InitPath(); //init map ig?
    }

    private static void InitPath(){
        
    }

    public Command getAuto(){
        //TODO: calculate nearest rotation need to be in to head
        return new ParallelDeadlineGroup(
            actionManager.getBestPoint().getCommand(),
            new SequentialCommandGroup());
    }

    //TODO: is obstacle on da way of traj? -> yes then pull sum random pose2d waypoints along da edges of da obstacle. 
    //TODO: find ways to check obstacle on traj
    //TODO: find ways to extends obstacle based on robot yaw + radius (in case robot aint symmetrical)
    //TODO: find ways to find waypoints that form the shortest traj

    //ik wut to do but idfk how to implement SHIT atm - ly (8/5)
}
    

