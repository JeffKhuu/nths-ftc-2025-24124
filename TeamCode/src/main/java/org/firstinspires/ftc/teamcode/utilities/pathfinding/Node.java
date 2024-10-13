package org.firstinspires.ftc.teamcode.utilities.pathfinding;

import com.acmerobotics.roadrunner.Vector2d;

public class Node {
    Vector2d coordinate;
    boolean isObstacle;

    int gCost;
    int hCost;
    int fCost = gCost + hCost;

    private Node(Vector2d coordinate, boolean isObstacle){
        this.coordinate = coordinate;
        this.isObstacle = isObstacle;
    };

    public static NodeBuilder Builder(int x, int y){
        return new NodeBuilder(x, y);
    }

    public static class NodeBuilder {
        private final Vector2d coordinate;
        private boolean obstructed = false;
        public NodeBuilder(int x, int y){
            coordinate = new Vector2d(x, y);
        }

        public NodeBuilder setObstructed(boolean obstructed){
            this.obstructed = obstructed; return this;
        }

        public Node build(){
            return new Node(coordinate, obstructed);
        }

    }
}


