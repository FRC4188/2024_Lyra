package lib.io.values;

interface IOValue<E> {
    public E get();

    public void set(E value);
}