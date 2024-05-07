package lib.logging.values;

import java.util.function.BiConsumer;

public class LoggedValue<E> extends Value<E> {
    private final BiConsumer<String, E> loggingFunction;

    public LoggedValue(String name, BiConsumer<String, E> loggingFunction) {
        super(name);

        this.loggingFunction = loggingFunction;
    }


    public LoggedValue(String path, String name, BiConsumer<String, E> loggingFunction) {
        super(path, name);

        this.loggingFunction = loggingFunction;
    }
    
    @Override
    public void set(E value) {
        if (!this.value.get().equals(value))
            loggingFunction.accept(super.name, value);
        
        super.set(value);
    }
}
