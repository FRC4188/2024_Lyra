package lib.logging.values;

import java.io.File;
import java.util.concurrent.atomic.AtomicReference;

public class Value<E> {
    protected AtomicReference<E> value;

    protected final String name;

    public Value(String name) {
        this.name = name;

        value = new AtomicReference<E>();
    }

    public Value(String path, String name) {
        this(String.format("%s%s%s", path, File.separator, name));
    }

    public synchronized E get() {
        return value.get();
    }

    public synchronized void set(E value) {
        this.value.set(value);
    }
}