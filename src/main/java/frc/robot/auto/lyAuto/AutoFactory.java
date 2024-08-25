package frc.robot.auto.lyAuto;

import java.util.List;

import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.lyAuto.task.ActionManager;

public class AutoFactory {
    private static AutoFactory instance;
    private ActionManager actionManager;

    // public static PathFinder getInstance(){
    //     if(instance == null) instance = new PathFinder();
    //     return instance;
    // }
    private AutoFactory(ActionManager actionManager) {
        this.actionManager = actionManager;
        InitPath();
    }

    public static void InitPath(){
        
    }
    //TODO: is obstacle on da way of traj? -> yes then pull sum random pose2d waypoints along da edges of da obstacle. 
    //TODO: find ways to check obstacle on traj
    //TODO: find ways to extends obstacle based on robot yaw + radius (in case robot aint symmetrical)
    //TODO: find ways to find waypoints that form the shortest traj

    //ik wut to do but idfk how to implement SHIT atm - ly (8/5)

    
}
