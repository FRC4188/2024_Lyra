package frc.robot.auto.lyAuto.task;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.FollowPath;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.groups.FeedIntake;
import frc.robot.commands.groups.ShootOnReady;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.sensors.Sensors;

import static frc.robot.Constants.robot.*;

public class Action {
    // public boolean isAvailable();
    
    // public void setSetpoint(String key, DoubleSupplier value);

    // public Translation2d getGoalPoint();

    // public Map<String, DoubleSupplier> getRequirement();

    // public Map<String, DoubleSupplier> getSetpoint();
    public Task task;
    private Node no;


    public Action(Task task) {
        this.task = task;
        this.no = no;
    }


    public class ActionBenefit{
        public static final int SPEAKER = 5; 
        public static final int INTAKE = 1;
        public static final int PASS = 2;

    }

    public enum Task{
        TRAVEL, INTAKE, SHOOT;
    }

    public STATE getPrecondState(){
        switch(task){
            case INTAKE:
                return STATE.UNLOADED;
            case SHOOT:
                return STATE.LOADED;
            default:
                return getRobotState();
        }
    }

    public double getBenefits(){
        switch(task){
            case  INTAKE:
                return 1;
            case SHOOT:
                return 5;
            default:
                return 0;
        }
    }

    public double getExecRadius(){
        switch(task){
            case INTAKE:
                return A_CROSSLENGTH / 2; //TODO: CHECK
            case SHOOT:
                return 0;  //TODO: CHANGE
            default:
                return 0;
        }
    }
}
