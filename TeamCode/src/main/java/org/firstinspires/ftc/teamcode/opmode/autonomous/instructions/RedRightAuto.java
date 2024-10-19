package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RedRightAuto extends AutoInstructions {
    RedRightAuto(LinearOpMode opMode) {
        super(opMode);
    }
    Pose2d startPose = new Pose2d(new Vector2d(-24, -72), 90);

    @Override
    public void execute() {
        opMode.waitForStart();



    }

    @Override
    public Pose2d getPose() {
        return startPose;
    }
}
