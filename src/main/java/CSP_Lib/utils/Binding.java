package CSP_Lib.utils;

import java.util.LinkedList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotControlState;

public class Binding {
    private Trigger trigger;
    private final LinkedList<RobotControlState> states = new LinkedList<>();

    public Binding(Trigger trigger, Command onTrue, Command onFalse) {
        this.trigger = trigger.and(() -> states.contains(Robot.getState()))
            .onTrue(onTrue).onFalse(onFalse);
    }

    public Binding addState(RobotControlState state) {
        states.add(state);
        return this;
    }
}