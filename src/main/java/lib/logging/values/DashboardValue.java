package lib.logging.values;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardValue<E> extends Value<E> {

    private E lastSent;

    private final Notifier dashboardNotifier = new Notifier(() -> {
        E value = this.value.get();

        if (!value.equals(lastSent))
            SmartDashboard.putString(name, value.toString());
        
        lastSent = value;
    });

    public DashboardValue(String name, double updatePeriod) {
        super(name);

        dashboardNotifier.startPeriodic(0);
    }

    public DashboardValue(String name) {
        this(name, 0.1);
    }
}
