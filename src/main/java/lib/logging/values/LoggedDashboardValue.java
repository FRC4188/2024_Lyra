package lib.logging.values;

import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoggedDashboardValue<E> extends LoggedValue<E> {
    private E lastSent;

    private final Notifier dashboardNotifier = new Notifier(() -> {
        E value = this.value.get();

        if (!value.equals(lastSent))
            SmartDashboard.putString(name, value.toString());
        
        lastSent = value;
    });

    public LoggedDashboardValue(String name, BiConsumer<String, E> loggingFunction, double period) {
        super(name, loggingFunction);

        dashboardNotifier.startPeriodic(period);
    }

    public LoggedDashboardValue(String path, String name, BiConsumer<String, E> loggingFunction, double period) {
        super(path, name, loggingFunction);

        dashboardNotifier.startPeriodic(period);
    }

    public LoggedDashboardValue(String name, BiConsumer<String, E> loggingFunction) {
        this(name, loggingFunction, 0.1);
    }

    public LoggedDashboardValue(String path, String name, BiConsumer<String, E> loggingFunction) {
        this(path, name, loggingFunction, 0.1);
    }
}
