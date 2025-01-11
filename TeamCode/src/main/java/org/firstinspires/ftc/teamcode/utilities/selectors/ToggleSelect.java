package org.firstinspires.ftc.teamcode.utilities.selectors;

public class ToggleSelect<T> {

    private T selected;
    private final T primary;
    private final T secondary;

    public ToggleSelect(T primary, T secondary) {
        this.primary = primary;
        this.secondary = secondary;
    }

    public ToggleSelect<T> toggle() {
        selected = selected == primary ? secondary : primary;
        return this;
    }

    public T getSelected() {
        return selected;
    }
}