package org.firstinspires.ftc.teamcode.utilities;

/**
 * Looping list of objects with moveable index. When the index reaches the end of the list, it resets to the beginning
 *
 * @param <T> Type of object inside the carousel.
 */
public class CarouselSelect<T> {
    /*
        CarouselSelect representation for array of size four
        -----------------------------------------------------------------------------------
        |                 |                     |                    |                    |
        |                 |                     |                    |                    |
        |                 |                     |                    |                    |
        |     value0      |      > value1 <     |       value2       |       value3       |
        |                 |                     |                    |                    |
        |                 |                     |                    |                    |
        -----------------------------------------------------------------------------------
                                                                                ↑ when the index reaches here it starts back at value0
        currentIndex = 1 (2nd space)
        moveSelection(3) -> currentIndex = 0
     */

    private final T[] options;
    private int currentIndex = 0;

    /**
     * Creates a carousel selection of the generic type T.
     *
     * @param options List of any type. Recommended to be three or greater.
     */
    public CarouselSelect(T[] options) {
        this.options = options;
    }

    /**
     * @return Currently selected object inside the carousel.
     */
    public T getSelected() {
        return options[currentIndex];
    }

    /**
     * Move the index of the carousel by 1. If the index ends out of bounds of the array, it will loop back to the start.
     *
     * @return Returns the CarouselSelect object. Useful for chaining methods.
     */
    public CarouselSelect<T> moveSelection() {
        currentIndex = (currentIndex + 1) % options.length;
        return this;
    }

    /**
     * Move the index of the carousel by a specified amount. If the index ends out of bounds of the array, it will loop back to the start.
     *
     * @param amount Amount to be added to the index
     * @return Returns the CarouselSelect object. Useful for chaining methods.
     */
    public CarouselSelect<T> moveSelection(int amount) {
        currentIndex = (currentIndex + amount) % options.length;
        return this;
    }


}
