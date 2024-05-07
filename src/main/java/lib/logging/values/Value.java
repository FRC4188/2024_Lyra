package lib.logging.values;

import java.io.File;
import java.util.concurrent.atomic.AtomicReference;

public class Value<E> {
    protected AtomicReference<E> value;

    protected final String name;

    public Value(String name) {
        this.name = name;
    }

    public Value(String path, String name) {
        this(String.format("%s%s%s", path, File.separator, name));
    }

    public E get() {
        return value.get();
    }

    public void set(E value) {
        this.value.set(value);
    }
}