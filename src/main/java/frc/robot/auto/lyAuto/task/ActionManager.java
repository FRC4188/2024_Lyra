package frc.robot.auto.lyAuto.task;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import frc.robot.auto.lyAuto.task.*;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Sensors;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.lyAuto.task.Node;
import frc.robot.auto.lyAuto.utils.AllianceFlip;

import static frc.robot.auto.lyAuto.utils.FieldConstants.*;

public class ActionManager {
    PriorityQueue<Node> available = new PriorityQueue<Node>(Comparator.comparingDouble(Node::getWeight));
    PriorityQueue<Node> execute = new PriorityQueue<Node>(Comparator.comparingDouble(Node::getWeight));

    public ActionManager(){
        init();
    }

    private void init(){
        //TODO: fix rotation equation (im too lazy to check equations, basically just find the rotation the robot should be when intake)
        //TODO: also make sure theres no obstacles in the way of the robot when it goes to intake (sensors + default map??) idfk

        for(int i = 0; i < StagingLocations.centerlineTranslations.length; i++){
            var note = StagingLocations.centerlineTranslations[i];
            note = AllianceFlip.apply(note);

            Pose2d noteP = new Pose2d(note, new Rotation2d(Math.abs(note.getX()-Swerve.getInstance().getPose2d().getX()), Math.abs(note.getY()-Swerve.getInstance().getPose2d().getY())));
            Node node  = new Node(new Action(Action.Task.INTAKE), noteP);
            node.updateDashboard("CenterlineNote."+i);
            available.add(node);
        }
        for(int i = 0; i < StagingLocations.spikeTranslations.length; i++){
            var no = StagingLocations.spikeTranslations[i];
            no = AllianceFlip.apply(no);

            Pose2d noteP = new Pose2d(no, new Rotation2d(Math.abs(no.getX()-Swerve.getInstance().getPose2d().getX()), Math.abs(no.getY()-Swerve.getInstance().getPose2d().getY())));
            Node node  = new Node(new Action(Action.Task.INTAKE), noteP);
            node.updateDashboard("SpikeNote."+i);
            available.add(node);          
        }

        available.add(new Node(new Action(Action.Task.SHOOT))); //shoot preloaded note
    }  

    public void update(){
        for(Node no: available){
            no.update();
        }
        updateNote();
    }

    public void updateDashboard(){
        // Field2d field = new Field2d();
        // SmartDashboard.putData(field);
    }

    private void updateNote(){
        //TODO: research how april tag localization work then apply to note localization
        //e.g. get sensors to constantly check note locations, update weight if needed

    }

    public Node getBestPoint(){
        return execute.poll();
    }
    
}
