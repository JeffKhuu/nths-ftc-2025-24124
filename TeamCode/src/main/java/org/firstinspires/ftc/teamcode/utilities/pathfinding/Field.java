package org.firstinspires.ftc.teamcode.utilities.pathfinding;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;

import java.util.ArrayList;
import java.util.Objects;

/*
 * Standard FTC Field layout and code representation:
 *  Coordinates based off the official FTC coordinate system are denoted as: (0, 0)FTC
 *  Coordinates based off the implemented tile coordinate system are denoted as: (6,6)
 *
 *  From the perspective of the audience, +x is up, -x is down, +y is left, -y right
 *
 *        (60,60)FTC     1          2           3           4           5      (72, -72)FTC
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
 *    5  |(-60, 60) |         |           |           |           |           |
 *       |          |         |           |           |           |           |
 *       ---------------------------------------------------------------------- *(-72, -72)FTC
 *
 *                                    AUDIENCE
 */

public enum Field {
    INSTANCE;

    private final Node[][] map = new Node[FieldConstants.rows][FieldConstants.cols];
    private static final int INCHES_PER_TILE = -24;
    private static final int TILE_OFFSET = 60;

    Field() {
        for (int row = 0; row < FieldConstants.rows; row++) {
            for (int col = 0; col < FieldConstants.cols; col++) {
                map[row][col] = Node.Builder(row, col)
                        .setObstructed(FieldConstants.FIELD_MAP[row][col] == 'x') // If the tile is obstructed in our representation, block out it's corresponding node
                        .build();
            }
        }
    }

    public Node[][] getMap() {
        return map;
    }


    public Node getTile(int x, int y) {
        if (x < 0 || x > FieldConstants.cols - 1)
            return null;
        else if (y < 0 || y > FieldConstants.rows - 1)
            return null;

        return map[x][y];
    }

    public static Vector2d tileToFTC(Vector2d tileCoords) {
        return new Vector2d(
                (tileCoords.x * INCHES_PER_TILE) + TILE_OFFSET,
                (tileCoords.y * INCHES_PER_TILE) + TILE_OFFSET);
    }

    public static Vector2d FTCToTile(Vector2d FTCCoords) {
        return new Vector2d(
                (FTCCoords.x - 72) / INCHES_PER_TILE,
                (FTCCoords.y - 72) / INCHES_PER_TILE);
    }

    // TODO: Round to closest tile
    public static Vector2d roundToNearestTile(Vector2d tileCoords) {
        return new Vector2d((int) tileCoords.x, (int) tileCoords.y);
    }

    public ArrayList<Node> getNeightbours(Node node) {
        ArrayList<Node> neighbours = new ArrayList<>();
        for (int i = (int) (node.coordinate.x - 1); i <= node.coordinate.x + 1; i++) {
            for (int j = (int) node.coordinate.y - 1; j <= node.coordinate.y + 1; j++) {
                if (getTile(i, j) == null)
                    continue; // Skip if the tile doesn't exist
                else if (i == node.coordinate.x && j == node.coordinate.y || Objects.requireNonNull(getTile(i, j)).isObstacle)
                    continue; // Skip the center tile and obstructed tiles

                neighbours.add(getTile(i, j));
            }
        }
        return neighbours;
    }
}
