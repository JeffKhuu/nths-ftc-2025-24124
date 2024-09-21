package org.firstinspires.ftc.teamcode.opmode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

import java.util.Objects;

@Autonomous(name = "0+0 Auto (Blue Alliance Left)", preselectTeleOp = "Main Teleop")
@AutonomousEx(preload = 0, cycles = 0, position = FieldConstants.START_POSITION.BLUE_ALLIANCE_LEFT)
public class AutoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        
    }
}
