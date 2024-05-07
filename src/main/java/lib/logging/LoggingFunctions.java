package lib.logging;

import java.util.function.BiConsumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.WPISerializable;

public final class LoggingFunctions {
    
    public static BiConsumer<String, Double> numberLogger() {
        return (String name, Double value) -> {
            Logger.recordOutput(name, value);
        };
    }


    public static BiConsumer<String, WPISerializable> wpiLogger() {
        return (String name, WPISerializable value) -> {
            Logger.recordOutput(name, value);
        };
    }


    public static BiConsumer<String, String> stringLogger() {
        return (String name, String value) -> {
            Logger.recordOutput(name, value);
        };
    }
}
