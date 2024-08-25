package frc.robot.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;

public class trajectories {
    /*Mech Advantage auto structure:
     * -> full reliance on wpilib not pathplaner or choreo
     * -> make trajs from code with Pose2d and helpers from their own lib
     * -> use drivetrain state to change max vel + what to do
     * 
     * -> each auto is a command, with following traj command parallel with 
     * other commands that usually act as event markers
     * -> used timer + expected "intake time" to determine when to intake notes
     * -> used enum starting pose to change intake time 
     */

     /*Citrus 2023 auto structure:
      * -> used pathweaver + endeffector
      -> each auto is an "Automode base" that run each command down the line 
      (no sequencing command, just going down each line of their actions in a function)
      */

      //TODO: make a holonomic drive for auto (almost all teams researched make a class for "holonomic drivetrain")
      //TODO: add adaptable autos based on note detection maybe? need more research
      //TODO: add more  "stops" in front of notes if have note detection (for seeing if there's a note + change traj if theres none)
      //TODO: make minimal amount of autos for max adjustable paths for customization in code 
    
    public static final ChoreoTrajectory Pcs5 = Choreo.getTrajectory("5 piece");
    public static final ChoreoTrajectory Src3pcs = Choreo.getTrajectory("Source 3 mid");
    public static final ChoreoTrajectory Spike4pcs = Choreo.getTrajectory("3rd spike");

    //Choreo can split trajectories into segments from where stop points at, code exemplar:
    public static final PathPlannerPath x = PathPlannerPath.fromChoreoTrajectory("choreoName.segmentNumber");

}

