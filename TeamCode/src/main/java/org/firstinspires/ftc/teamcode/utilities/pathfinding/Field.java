package org.firstinspires.ftc.teamcode.utilities.pathfinding;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;

/*
 * Standard FTC Field layout and code representation:
 *  Coordinates based off the official FTC coordinate system are denoted as: (0, 0)FTC
 *  Coordinates based off the implemented tile coordinate system are denoted as: (6,6)
 *
 *  From the perspective of the audience, +x is up, -x is down, +y is left, -y right
 *
 *  (72,72)FTC  0        1          2           3           4           5      (72, -72)
 *       *----------*---------*-----------*-----------*-----------*-----------*
 *       |          |         |           |           |           |           |
 *     0 |          | (60,36) |           |           |           |           |
 *       |          |         |           |           |           |           |
 *       ----------------------------------------------------------------------
 *       |          |         |           |           |           |           |
 *     1 |          |         |           |           |           |           |
 *       |          |         |           |           |           |           |
 *       ----------------------------------------------------------------------
 *       |          |         |           |           |           |           |
 *     2 |          |         | (+x, +y)  | (+x, -y)  |           |           |
 *       |          |         |           |           |           |           |
 * BLUE  ------------------------------(0,0)FTC--------------------------------   RED
 *       |          |         |           |           |           |           |
 *    3  |          |         | (-x, +y)  | (-x, -y)  |           |           |
 *       |          |         |           |           |           |           |
 *       ----------------------------------------------------------------------
 *       |          |         |           |           |           |           |
 *    4  |          |         |           |           |           |           |
 *       |          |         |           |           |           |           |
 *       ---------------------------------------------------------*------------
 *       |          |         |           |           |           |           |
 *    5  |          |         |           |           |           |           |
 *       |          |         |           |           |           |           |
 *       ---------------------------------------------------------------------- *(-72, -72)FTC
 *
 *                                    AUDIENCE
 */


// TODO: Make a singleton
// TODO: Implement custom map interpretation from FieldConstants.java
public enum Field {
    INSTANCE;

    public final Node[][] map = new Node[FieldConstants.rows][FieldConstants.cols];
    private static final int INCHES_PER_TILE = -24;
    private static final int TILE_OFFSET = 60;

    Field() {
        for(int row = 0; row < FieldConstants.rows; row++){
            for(int col = 0; col < FieldConstants.cols; col++){
                map[row][col] = Node.Builder(row, col)
                        .setObstructed(FieldConstants.FIELD_MAP[row][col] == 'x') // If the tile is obstructed on our representation, block out it's corresponding node
                        .build();
            }
        }
    }

    public Node[][] getMap() {
        return map;
    }

    public static Vector2d TileToFTC(Vector2d tileCoords){
        return new Vector2d(
                (tileCoords.x * INCHES_PER_TILE) + TILE_OFFSET,
                (tileCoords.y * INCHES_PER_TILE) + TILE_OFFSET);
    }

    public static Vector2d FTCToTile(Vector2d FTCCoords){
        return new Vector2d(
                (FTCCoords.x - 72) / INCHES_PER_TILE,
                (FTCCoords.y - 72) / INCHES_PER_TILE);
    }
}
