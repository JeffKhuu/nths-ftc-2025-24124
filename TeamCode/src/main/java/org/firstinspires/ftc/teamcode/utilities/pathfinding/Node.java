package org.firstinspires.ftc.teamcode.utilities.pathfinding;

import com.acmerobotics.roadrunner.Vector2d;

public class Node {
    public Vector2d coordinate;
    boolean isObstacle;

    private Node parent; // Pointer to another node, used in order to reconstruct our path
    private int gCost = 0; // Distance from a specified starting node
    private int hCost = 0; // Distance to a specified end node

    private Node(Vector2d coordinate, boolean isObstacle) {
        this.coordinate = coordinate;
        this.isObstacle = isObstacle;
    }

    public void setParent(Node parent) {
        this.parent = parent;
    }

    public void setG(int gCost) {
        this.gCost = gCost;
    }

    public void setH(int hCost) {
        this.hCost = hCost;
    }

    public Node getParent() {
        return parent;
    }

    public int getG() {
        return gCost;
    }

    public int getF() {
        return gCost + hCost;
    } // Our F cost is defined as the combined costs

    /**
     * Distance calculation between two tiles on a square based grid.
     * Based off TaroDev's A* Pathfinding distance algorithm.
     *
     * @param other The ending tile.
     * @return Returns the distance between the two tiles as a integer.
     * @see <a href=https://github.com/Matthew-J-Spencer/Pathfinding/blob/main/_Scripts/Tiles/SquareNode.cs>TaroDev Sample Code</a>
     */
    public int getDistance(Node other) {
        if (other == null) return -1; // Return a distance of -1 if the other tile does not exist.
        Vector2d dist = new Vector2d(Math.abs((int) coordinate.x - (int) other.coordinate.x), Math.abs((int) coordinate.y - (int) other.coordinate.y));

        double lowest = Math.min(dist.x, dist.y);
        double highest = Math.max(dist.x, dist.y);

        double horizontalDist = highest - lowest;
        return (int) (lowest * 14 + horizontalDist * 10);
    }

    public static NodeBuilder Builder(int x, int y) {
        return new NodeBuilder(x, y);
    }

    public static class NodeBuilder {
        private final Vector2d coordinate;
        private boolean obstructed = false;

        public NodeBuilder(int x, int y) {
            coordinate = new Vector2d(x, y);
        }

        public NodeBuilder setObstructed(boolean obstructed) {
            this.obstructed = obstructed;
            return this;
        }

        public Node build() {
            return new Node(coordinate, obstructed);
        }

    }
}


