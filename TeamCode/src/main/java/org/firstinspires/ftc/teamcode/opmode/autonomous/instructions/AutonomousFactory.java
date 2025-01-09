package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.FieldConstants.ALLIANCE;
import org.firstinspires.ftc.teamcode.constants.FieldConstants.START_POSITION;

public class AutonomousFactory {
    LinearOpMode opMode;

    public AutonomousFactory(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public AutoInstructions buildAuto(ALLIANCE alliance, START_POSITION position) {
        if (alliance == ALLIANCE.RED && position == START_POSITION.ALLIANCE_LEFT)
            return new RedLeftAuto(opMode); // RED ALLIANCE LEFT
        else if (alliance == ALLIANCE.RED && position == START_POSITION.ALLIANCE_RIGHT)
            return new RedRightAuto(opMode); // RED ALLIANCE RIGHT
        else if (alliance == ALLIANCE.BLUE && position == START_POSITION.ALLIANCE_LEFT)
            return new BlueLeftAuto(opMode); // BLUE ALLIANCE LEFT
        else
             return new BlueRightAuto(opMode); // BLUE ALLIANCE RIGHT
    }
}
