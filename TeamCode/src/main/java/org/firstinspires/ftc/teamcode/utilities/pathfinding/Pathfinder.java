package org.firstinspires.ftc.teamcode.utilities.pathfinding;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Set;

public class Pathfinder {
    /**
     * A* Pathfinding implementation which returns the shortest path from one tile to another.
     *
     * @param start Node representing the starting tile
     * @param end   Node representing the ending tile to pathfind towards
     * @return Returns an array list of tiles with the shortest path from start to end, including the start and end.
     * @see <a href=https://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html>Stanford Resource on A* Pathfinding</a>
     */
    public ArrayList<Node> findPath(Node start, Node end) {
        PriorityQueue<Node> frontier = new PriorityQueue<>(new FCostComparator());
        Set<Node> interior = new HashSet<>();
        //setCosts(start, start, end);
        frontier.add(start);

        while (!frontier.isEmpty()) {
            Node currentTile = frontier.poll(); // Dequeues the best tile from the frontier
            if (currentTile != null) {

                // If we have reached the end tile, reconstruct the path and return
                if (currentTile == end) {
                    Node pathTile = end;
                    ArrayList<Node> path = new ArrayList<>();

                    while (pathTile != start) {
                        path.add(pathTile);
                        pathTile = pathTile.getParent();
                    }
                    path.add(start);
                    Collections.reverse(path);

                    ArrayList<Node> new_path = optimizePath(path);
                    for (Node tile : new_path) {
                        System.out.println(tile.coordinate.x + " " + tile.coordinate.y);
                    }

                    return path;
                }

                // Add the current tile to the set of explored tiles
                interior.add(currentTile);

                // Investigate each neighbour of the current tile
                for (Node neighbour : Field.INSTANCE.getNeightbours(currentTile)) {
                    int cost = currentTile.getG() + currentTile.getDistance(neighbour); // Calculate an estimated cost to get to the tile

                    if (frontier.contains(neighbour) && cost < neighbour.getG()) // Remove the neighbour tile from the queue if we have found a better path, to be able to be requeued
                        frontier.remove(neighbour);
                    if (interior.contains(neighbour) && cost < neighbour.getG()) // Remove the neighbour tile from set of explored tiles if we have found a better path, to be able to be requeued
                        interior.remove(neighbour);
                    if (!frontier.contains(neighbour) && !interior.contains(neighbour)) {
                        neighbour.setG(cost); // Calculate estimated G and H costs for the tile
                        neighbour.setH(neighbour.getDistance(end));
                        neighbour.setParent(currentTile); // Set a parent tile to be able to reconstruct our path after reaching the end

                        frontier.add(neighbour); // Add the neighbour tile to be explored
                    }
                }
            }
        }
        return null; // Return null if there is no path.
    }

    public ArrayList<Node> optimizePath(ArrayList<Node> path) {
        /*

        0,0 -> 1,0 -> 2,0 -> 3,0 -> 4,0 -> 4,1 -> 5,2 -> 6,2 -> 7,3
        0,0 -> 4,0 -> 4,1 -> 5,2 -> 6,2 -> 7,3

        Algorithm:
         Loop throuigh all nodes in a path:
             if(
         */
        ArrayList<Node> optimized = new ArrayList<>();
        Node first = path.get(0);
        Node last = path.get(path.size() - 1);

        optimized.add(first);
        for (Node tile : path) {
            if (tile == last) continue;
            Node next = path.get(path.indexOf(tile) + 1);
            System.out.println((tile.coordinate.x - next.coordinate.x != 0) + " " + (tile.coordinate.y - next.coordinate.y == 0) + " | " + tile.coordinate.x + " " + tile.coordinate.y + " & " + next.coordinate.x + " " + next.coordinate.y);

            if (tile.coordinate.x - next.coordinate.x != 0 && tile.coordinate.y - next.coordinate.y != 0) {
                optimized.add(tile);
                optimized.add(next);
            }
        }
        if (!optimized.contains(last)) optimized.add(last);

        return optimized;
    }

    //region Testing
    public static void main(String[] args) {
        for (Node[] col : Field.INSTANCE.getMap()) { // Testing the entire map
            for (Node tile : col)
                System.out.printf("(" + (int) tile.coordinate.x + "," + (int) tile.coordinate.y + ") ");
            System.out.print("\n");
        }

        Node tile = Field.INSTANCE.getTile(1, 0); // Testing retrieving neighbours of a tile
        if (tile != null) {
            for (Node neighbor : Field.INSTANCE.getNeightbours(tile))
                System.out.println("Neighbour of (1,0): " + Field.tileToFTC(neighbor.coordinate));
        }

        System.out.println("Distance from (1,0) to (1,4): " + Field.INSTANCE.getMap()[1][0].getDistance(Field.INSTANCE.getMap()[1][4])); // Testing distances Expected: 44

        Pathfinder pathfinder = new Pathfinder(); // Testing Pathfinder
        System.out.println("Shortest path from (0,0) to (4,1): ");
        for (Node path : pathfinder.findPath(Field.INSTANCE.getTile(0, 0), Field.INSTANCE.getTile(4, 1))) {
            Vector2d ftcPath = Field.tileToFTC(path.coordinate);
            System.out.println((int) path.coordinate.x + ", " + (int) path.coordinate.y + " (" + (int) ftcPath.x + "," + (int) ftcPath.y + ")FTC");
        }
    }
    //endregion
}

/**
 * Comparator used by the pathfinder to compare relative F costs for use in a priority queue
 */
class FCostComparator implements Comparator<Node> {
    @Override
    public int compare(Node node1, Node node2) {
        return Integer.compare(node1.getF(), node2.getF());
    }
}
