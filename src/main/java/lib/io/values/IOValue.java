package lib.io.values;

import java.io.File;
import java.util.concurrent.atomic.AtomicReference;

public class IOValue<E> {
    protected AtomicReference<E> value;

    protected final String name;

    public IOValue(String name) {
        this.name = name;
    }

    public IOValue(String path, String name) {
        this.name = String.format("%s%s%s", path, File.separator, name);
    }

    public E get() {
        return value.get();
    }

    public void set(E value) {
        this.value.set(value);
    }
}