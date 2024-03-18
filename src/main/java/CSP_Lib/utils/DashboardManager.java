package CSP_Lib.utils;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Automates and optimizes many {@link SmartDashboard} interactions.
 * Only works with doubles.
 */
public class DashboardManager {

    private static final double DEFAULT_PERIOD = 0.1;

    private static HashMap<String, Double> outputMap = new HashMap<>();

    private static final AtomicReference<HashMap<String, Double>> inputRef =
     new AtomicReference<HashMap<String,Double>>(new HashMap<String, Double>());

    private static final LinkedList<NetworkTableListener> listeners = new LinkedList<>();

    private static Notifier runner = new Notifier(() -> update());

    /**
     * Begins the manager running at the default period.
     */
    public static void start() {
        start(DEFAULT_PERIOD);
    }

    /**
     * Begins the manager at a specified period.
     * @param period The period of the manager's updates in seconds between refreshes.
     */
    public static void start(double period) {
        runner.startPeriodic(period);
    }

    private static void update() {
        for (String name : outputMap.keySet()) {
            SmartDashboard.putNumber(name, outputMap.get(name));
        }

        outputMap = new HashMap<>();
    }

    private static void addEntry(String name, double value) {
        SmartDashboard.putNumber(name, value);
        outputMap.put(name, value);
    }

    /**
     * Send an output value to {@link SmartDashboard}, whether it exists or not.
     * @param name The string name identifying this value.
     * @param value The value being sent.
     */
    public static void putValue(String name, double value) {
        if (outputMap.containsKey(name))
            outputMap.put(name, value);
        else addEntry(name, value);
    }

    private static synchronized void innerFunction(NetworkTableEvent event, String name) {
        inputRef.get().put(name, event.valueData.value.getDouble());
    }

    /**
     * Start listening for input for a certain field.
     * @param name The string name of the field.
     * @param value The default value of the field before any input is given.
     */
    public static void addNumberInput(String name, double value) {
        listeners.add(
            NetworkTableListener.createListener(
                SmartDashboard.getEntry(name),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                (NetworkTableEvent event) -> innerFunction(event, name)
            ));
        
        inputRef.get().put(name, value);

        SmartDashboard.putNumber(name, value);
    }

    /**
     * Get the most recent input value of a field.
     * @param name The string name of the field
     * @return The most recent observed value of the field.
     */
    public static double readNumberInput(String name) {
        return inputRef.get().get(name);
    }
}
