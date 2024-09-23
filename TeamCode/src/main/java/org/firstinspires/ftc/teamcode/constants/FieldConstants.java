package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.roadrunner.Pose2d;

public class FieldConstants {
    /**
     * Possible starting positions of the robot used in positioning logic.
     * LEFT and RIGHT are relative from the Red/Blue Alliance Areas.
     */
    public enum ALLIANCE {
        RED,
        BLUE
    }

    public enum START_POSITION {
        ALLIANCE_LEFT,
        ALLIANCE_RIGHT
    }

    public static final char[][] mapKey = {
            {'o', 'o', 'o', 'o', 'o', 'o'},
            {'o', 'o', 'o', 'o', 'o', 'o'},
            {'o', 'o', 'x', 'x', 'o', 'o'},
            {'o', 'o', 'x', 'x', 'o', 'o'},
            {'o', 'o', 'o', 'o', 'o', 'o'},
            {'o', 'o', 'o', 'o', 'o', 'o'},
    };

    private static Pose2d lastSavedPose = new Pose2d(0, 0, 0);

    public static void savePose(Pose2d pose) {
        lastSavedPose = pose;
    }

    public static Pose2d getLastSavedPose() {
        return lastSavedPose;
    }

}
