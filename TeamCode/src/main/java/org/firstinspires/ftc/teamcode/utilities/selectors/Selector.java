package org.firstinspires.ftc.teamcode.utilities.selectors;

public interface Selector<T> {
    T getSelected();

    T[] getAllOptions();
}
