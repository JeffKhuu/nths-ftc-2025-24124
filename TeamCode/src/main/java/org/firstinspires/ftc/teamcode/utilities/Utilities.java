package org.firstinspires.ftc.teamcode.utilities;

public class Utilities {
    /**
     * Determine whether a given value is between the ranges of a given min and max
     * @param value The value
     * @param min The lower bound of the range
     * @param max The upper bound of the range
     * @return True, if the value is between the min and max, false otherwise.
     */
    public static boolean isBetween(int value, double min, double max){
        return value >= min && value <= max;
    }

}
