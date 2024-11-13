import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.utilities.pathfinding.Field;
import org.firstinspires.ftc.teamcode.utilities.pathfinding.Node;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;

class PathfinderTest {
    @Test
    void testFindingNeighbours() {
        Node tile = Field.INSTANCE.getTile(0, 0);
        ArrayList<Node> neighbors = new ArrayList<>(Arrays.asList(Field.INSTANCE.getTile(0, 1), Field.INSTANCE.getTile(1, 0), Field.INSTANCE.getTile(1, 1)));
        assert tile != null;
        assertEquals(Field.INSTANCE.getNeightbours(tile), neighbors);
    }
}