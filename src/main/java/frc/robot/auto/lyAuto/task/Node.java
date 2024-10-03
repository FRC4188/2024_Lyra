package frc.robot.auto.lyAuto.task;

import static frc.robot.Constants.robot.robotState;

import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.util.List;
import java.util.Observable;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.lyAuto.utils.FieldConstants;
import frc.robot.commands.drivetrain.FollowPath;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.groups.FeedIntake;
import frc.robot.commands.groups.ShootOnReady;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.sensors.Sensors;

public class Node {
    private final Action action;
    private Pose2d location;
    private Boolean isActive = true;
    private double weight = 0;
    private double execRadius;
    // private PropertyChangeSupport support = new PropertyChangeSupport(this);
    

    public Node(Action action, Pose2d location) {
        this.location = location;
        this.action = action;
        this.execRadius = action.getExecRadius();
        // this.support = new PropertyChangeSupport(this);
    }

    public Node(Action action){
        this.action = action;
        location = Swerve.getInstance().getPose2d();
        this.execRadius = action.getExecRadius();
        // this.support = new PropertyChangeSupport(this);
    }

    //Node is active if robot is in range of the action and the preconditions are met
    private void updateActive(){
        if (checkLocation() && robotState == action.getPrecondState()) { //TODO: CHANGE DA CONDITION PLS
            isActive = true;
        }else   
            isActive = false;
    }

    public void update(){
        updateActive();
        /* if sensors detect changes (note and robot detect) = apply changes in weight + exec time */
    }

    public void updateDashboard(String name){
        setPriority(SmartDashboard.getNumber(name + " priority", 0.0));
    }

    private void checkRisk(){
        double bottomX = 0;
        if(Sensors.getInstance().getAllianceColor() == DriverStation.Alliance.Blue)
            bottomX = FieldConstants.StagingLocations.centerlineX;
        
       
        if(Math.max(bottomX, Swerve.getInstance().getPose2d().getX()) == Math.min(bottomX + FieldConstants.wingX, Swerve.getInstance().getPose2d().getX()))
            weight = -50;
    }

    public double getDistanceFromCurr(){
        return location.getTranslation().getDistance(Swerve.getInstance().getPose2d().getTranslation());
    }

    public double getExecTime(){
        Trajectory traj = TrajectoryGenerator.generateTrajectory(List.of(Swerve.getInstance().getPose2d(), location), 
            new TrajectoryConfig(Constants.drivetrain.MAX_VELOCITY / 4, Constants.drivetrain.MAX_ACCEL / 4));
        return traj.getTotalTimeSeconds();
    }

    /**
     * Get the weight of importance of going from current (initial) pose to desired pose. (most important = lowest weight)
     * Higher time = higher weight, larger distance = smaller weight (TODO: MAKE THE EQUATION FOR THIS PLS)
     * @param initial pose
     * @param goal pose
     * @return  weight
     */
    public void updateWeight(){
        checkRisk();
        weight = weight - action.getBenefits() + getExecTime() + getDistanceFromCurr(); //TODO: change to better equation, im suck at making shit up
    } 

    public boolean isActive() {
        return isActive;
    }

    public Pose2d getLocation(){
        return location;
    }

    public double getWeight(){
        return weight;
    }

    public Action getAction(){
        return action;
    }

    public void setLocation(Pose2d location){
        if(this.location != location){
            this.location = location;
            updateWeight();
        }
    }

    /**
     * for later use if auto path generation works good.
     * for strat, use this to indicate which note should be prioritized first
     * higher priority value will lead to game element more likely to execute first
     * negative value will result in game element being last in line for execution
     * @param priority value
     */
    private void setPriority(double priority){
        weight -= priority;
    }

    //TODO: depends on the task(command) the execution range of the robot is different (shoot range is bigger than intake range)

    private boolean checkLocation(){
        if(execRadius == 0) return true;
        Pose2d robot = Swerve.getInstance().getPose2d();
        return Math.abs(robot.getX()-location.getX()) <= execRadius
            && Math.abs(robot.getY()-location.getX()) <= execRadius;
    }

     /**
     * @return
     */
    public Command getCommand() {
        switch (action.task) {
            case TRAVEL: //TODO: calculate nearest rotation according to robot pose, and implement tolerance in term of exec radius
                return new FollowPath( TrajectoryGenerator.generateTrajectory(
                    Swerve.getInstance().getPose2d(), null, location,
                    new TrajectoryConfig(Constants.drivetrain.MAX_VELOCITY / 4, Constants.drivetrain.MAX_ACCEL / 4)),
                    location.getRotation()); //TODO: add trajectory and figure out how to calculate headings for note
            
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

    // public void addPropertyChangeListener(PropertyChangeListener e){
    //     support.addPropertyChangeListener(e);
    // }

    // public void removePropertyChangeListener(PropertyChangeListener e){
    //     support.removePropertyChangeListener(e);
    // }
}