package CSP_Lib.utils;

import java.util.LinkedList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotControlState;

/**
 * Constrains behaviors of a trigger to only while the robot is in a certain state.
 */
public class Binding {
    private Trigger trigger;
    private final LinkedList<RobotControlState> states = new LinkedList<>();

    /**
     * Create a new binding.
     * @param trigger The trigger which the binding is tied to.
     * @param onTrue The {@link Command} for the onTrue behvior of the trigger.
     * @param onFalse The {@link Command} for the onFalse behvior of the trigger.
     */
    public Binding(Trigger trigger, Command onTrue, Command onFalse) {
        this.trigger = trigger.and(() -> states.contains(Robot.getState()));
        
        if (onTrue != null)
            trigger.onTrue(onTrue);
        if (onFalse != null)
            trigger.onFalse(onFalse);
    }

    /**
     * Adds a new state this binding applies in.
     * @param state The {@link RobotControlState} this binding will be applied in.
     * @return The resulting binding which applies in all the added states before but also in the added state.
     */
    public Binding addState(RobotControlState state) {
        states.add(state);
        return this;
    }
}