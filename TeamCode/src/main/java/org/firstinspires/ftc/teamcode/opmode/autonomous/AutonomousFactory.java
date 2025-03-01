package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.FieldConstants.ALLIANCE;
import org.firstinspires.ftc.teamcode.constants.FieldConstants.START_POSITION;
import org.firstinspires.ftc.teamcode.opmode.autonomous.instructions.BlueLeftAuto;
import org.firstinspires.ftc.teamcode.opmode.autonomous.instructions.BlueRightAuto;
import org.firstinspires.ftc.teamcode.opmode.autonomous.instructions.RedLeftAuto;
import org.firstinspires.ftc.teamcode.opmode.autonomous.instructions.RedRightAuto;

public class AutonomousFactory {
    protected LinearOpMode opMode;

    private final AutoInstructions RED_LEFT, RED_RIGHT, BLUE_LEFT, BLUE_RIGHT;

    protected AutonomousFactory(LinearOpMode opMode) {
        this.opMode = opMode;

        RED_LEFT = new RedLeftAuto(opMode);
        RED_RIGHT = new RedRightAuto(opMode);
        BLUE_LEFT = new BlueLeftAuto(opMode);
        BLUE_RIGHT = new BlueRightAuto(opMode);
    }

    public AutoInstructions buildAuto(ALLIANCE alliance, START_POSITION position) {
        if (alliance == ALLIANCE.RED && position == START_POSITION.ALLIANCE_LEFT)
            return RED_LEFT;
        else if (alliance == ALLIANCE.RED && position == START_POSITION.ALLIANCE_RIGHT)
            return RED_RIGHT;
        else if (alliance == ALLIANCE.BLUE && position == START_POSITION.ALLIANCE_LEFT)
            return BLUE_LEFT;
        else if (alliance == ALLIANCE.BLUE && position == START_POSITION.ALLIANCE_RIGHT)
            return BLUE_RIGHT;

        return null;
    }
}
