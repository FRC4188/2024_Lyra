package CSP_Lib.utils;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardManager {
    
    // private static DashboardManager instance = null;
    // public static synchronized DashboardManager getInstance() {
    //     if (instance == null) instance = new DashboardManager();
    //     return instance;
    // }

    private static final double DEFAULT_PERIOD = 0.1;

    private static final HashMap<String, Double> outputMap = new HashMap<>();

    private static final AtomicReference<HashMap<String, Double>> inputRef =
     new AtomicReference<HashMap<String,Double>>(new HashMap<String, Double>());

    private static final LinkedList<NetworkTableListener> listeners = new LinkedList<>();

    private static Notifier runner = new Notifier(() -> update());

    public static void start() {
        start(DEFAULT_PERIOD);
    }

    public static void start(double period) {
        runner.startPeriodic(period);
    }

    private static void update() {
        for (String name : outputMap.keySet()) {
            SmartDashboard.putNumber(name, outputMap.get(name));
        }
    }

    private static void addEntry(String name, double value) {
        SmartDashboard.putNumber(name, value);
        outputMap.put(name, value);
    }

    public static void putValue(String name, double value) {
        if (outputMap.containsKey(name))
            outputMap.put(name, value);
        else addEntry(name, value);
    }

    private static synchronized void innerFunction(NetworkTableEvent event, String name) {
        inputRef.get().put(name, event.valueData.value.getDouble());
        putValue("inner ran", 1.0);
    }

    public static void addNumberInput(String name, double value) {
        SmartDashboard.putNumber(name, value);

        listeners.add(
            NetworkTableListener.createListener(
                SmartDashboard.getEntry(name),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                (NetworkTableEvent event) -> innerFunction(event, name)
            ));
        
        inputRef.get().put(name, value);
    }

    public static double readNumberInput(String name) {
        return inputRef.get().get(name);
    }
}
