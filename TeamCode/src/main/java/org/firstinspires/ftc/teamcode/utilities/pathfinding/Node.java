package org.firstinspires.ftc.teamcode.utilities.pathfinding;

import com.acmerobotics.roadrunner.Vector2d;

public class Node {
    Vector2d coordinate;

    int gCost;
    int hCost;
    int fCost = gCost + hCost;

    public Node(int x, int y) {
        coordinate = new Vector2d(x, y);
    }
}
