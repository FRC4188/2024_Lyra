package lib.logging.values;

import java.util.EnumSet;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.logging.LoggingFunctions;

public class LoggedDashboardInput extends LoggedValue<Double> {

    private final NetworkTableListener listener;

    public LoggedDashboardInput(String name, double initialValue) {
        super(name, LoggingFunctions.numberLogger());
    

        SmartDashboard.putNumber(super.name, initialValue);
        set(initialValue);

        this.listener = NetworkTableListener.createListener(
            SmartDashboard.getEntry(super.name),
            EnumSet.of(Kind.kValueAll),
            (NetworkTableEvent event) -> {
                set(event.valueData.value.getDouble());
            }
        );
    }
    
    public LoggedDashboardInput(String path, String name, double initialValue) {
        super(path, name, LoggingFunctions.numberLogger());

        SmartDashboard.putNumber(super.name, initialValue);
        set(initialValue);

        this.listener = NetworkTableListener.createListener(
            SmartDashboard.getEntry(super.name),
            EnumSet.of(Kind.kValueAll),
            (NetworkTableEvent event) -> {
                set(event.valueData.value.getDouble());
            }
        );
    }
    
    public LoggedDashboardInput(String name) {
        this(name, 0.0);
    }

    public LoggedDashboardInput(String path, String name) {
        this(path, name, 0.0);
    }
}
