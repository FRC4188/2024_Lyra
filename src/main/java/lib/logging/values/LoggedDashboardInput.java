package lib.logging.values;

import java.util.EnumSet;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.logging.LoggingFunctions;

public class LoggedDashboardInput extends LoggedValue<Double> {

    public LoggedDashboardInput(String name, double initialValue) {
        super(name, LoggingFunctions.numberLogger());
    

        SmartDashboard.putNumber(super.name, initialValue);
        set(initialValue);

        NetworkTableListener.createListener(
            SmartDashboard.getEntry(super.name),
            EnumSet.of(Kind.kValueRemote),
            (NetworkTableEvent event) -> {
                set(event.valueData.value.getDouble());
            }
        );
    }
    
    public LoggedDashboardInput(String path, String name, double initialValue) {
        super(path, name, LoggingFunctions.numberLogger());

        SmartDashboard.putNumber(super.name, initialValue);
        set(initialValue);

        NetworkTableListener.createListener(
            SmartDashboard.getEntry(super.name),
            EnumSet.of(Kind.kValueRemote),
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
