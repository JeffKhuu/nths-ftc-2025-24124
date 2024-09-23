package org.firstinspires.ftc.teamcode.utilities.pathfinding;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.HashMap;
import java.util.Map;

/*
 * Standard FTC Field layout and code representation:
 *  Coordinates based off the official FTC coordinate system are denoted as: (0, 0)FTC
 *  Coordinates based off the implemented tile coordinate system are denoted as: (6,6)
 *
 *  From the perspective of the audience, +x is up, -x is down, +y is left, -y right
 *
 *  (72,72)FTC  6        5          4           3           2           1
 *       ----------------------------------------------------------------------
 *       |          |         |           |           |           |           |
 *     6 |          |         |           |           |           |           |
 *       |          |         |           |           |           |           |
 *       ----------------------------------------------------------------------
 *       |          |         |           |           |           |           |
 *     5 |          |         |           |           |           |           |
 *       |          |         |           |           |           |           |
 *       ----------------------------------------------------------------------
 *       |          |         |           |           |           |           |
 *     4 |          |         | (+x, +y)  | (+x, -y)  |           |           |
 *       |          |         |           |           |           |           |
 * BLUE  ------------------------------(0,0)FTC--------------------------------   RED
 *       |          |         |           |           |           |           |
 *    3  |          |         | (-x, +y)  | (-x, -y)  |           |           |
 *       |          |         |           |           |           |           |
 *       ----------------------------------------------------------------------
 *       |          |         |           |           |           |           |
 *    2  |          |         |           |           |           |           |
 *       |          |         |           |           |           |           |
 *       ----------------------------------------------------------------------
 *       |          |         |           |           |           |           |
 *    1  |          |         |           |           |           |           |
 *       |          |         |           |           |           |           |
 *       ---------------------------------------------------------------------- (-72, -72)FTC
 *
 *                                    AUDIENCE
 */


// TODO: Make a singleton
// TODO: Implement custom map interpretation from FieldConstants.java
public class Field {
    public Map<Vector2d, Node> map = new HashMap<>();

    /**
     * Creates a representation of an FTC Field following official FTC field coordinate system standards.
     * For use with Pathfinder class. (The center of the field is defined as zero in the x, and zero in the y.
     *
     * @param width  Width of the field in the number of foam tiles (Even numbers work best)
     * @param height Height of the field in the number of foam tiles (Even numbers work best)
     * @see <a href="https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html">Field Coordinate System</a>
     */
    public Field(int width, int height) {
        for (int i = -width / 2; i <= width / 2; i++) {
            for (int j = -height / 2; j <= -height / 2; j++) {
                if (i == 0 || j == 0)
                    continue; // Skip tiles (0, -j...j) and (-i...i, 0) because they don't exist in our representation

                map.put(new Vector2d(i, j), new Node(i, j));
            }
        }
    }

    /**
     * Overloaded constructor using the default FTC field dimensions. (6 foam tiles x 6 foam tiles) or (144in x 144in)
     * <p>
     * Creates a representation of an FTC Field following official FTC field coordinate system standards.
     * For use with Pathfinder class. (The center of the field is defined as zero in the x, and zero in the y.
     *
     * @see <a href="https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html">Field Coordinate System</a>
     */
    public Field() {
        for (int i = -3; i <= 3; i++) {
            for (int j = 3 / 2; j <= 3; j++) {
                map.put(new Vector2d(i, j), new Node(i, j));
            }
        }
    }
}
