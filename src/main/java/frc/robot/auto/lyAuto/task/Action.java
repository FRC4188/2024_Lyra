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


    public Action(Task task) {
        this.task = task;
    }


    public class ActionBenefit{
        public static final int SPEAKER = 5; 
        public static final int INTAKE = 1;
        public static final int PASS = 2;

    }

    public enum Task{
        TRAVEL, INTAKE, SHOOT;
    }

    /**
     * @return
     */
    public Command getCommand() {
        switch (task) {
            case TRAVEL:
                return new FollowPath(null, null); //TODO: add trajectory and figure out how to calculate headings for note
            
            case INTAKE:
                return new FeedIntake();

            case SHOOT:
                return new ConditionalCommand(
                    new ShootOnReady(() -> 0.0, () -> 0.0).withTimeout(1.5)
                                                .andThen(
                                                    new ConditionalCommand(
                                                        new FeedIntoShooter(12.0).withTimeout(0.9),
                                                        new SequentialCommandGroup(),
                                                        () -> Feeder.getInstance().isBroken())
                                                ),
                    new FeedIntake().until(() -> Feeder.getInstance().isBroken()).withTimeout(1.0).andThen(
                    new ShootOnReady(() -> 0.0, () -> 0.0).withTimeout(1.5)
                                                .andThen(
                                                    new ConditionalCommand(
                                                        new FeedIntoShooter(12.0).withTimeout(0.9),
                                                        new SequentialCommandGroup(),
                                                        () -> Feeder.getInstance().isBroken())
                                                )),
                    () -> Feeder.getInstance().isBroken()
                ).withTimeout(4.0);

            default:
                return new InstantCommand();
        }
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
