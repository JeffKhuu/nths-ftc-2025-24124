package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 0, cycles = 0)
public class RedRightAuto extends AutoInstructions {
    RedRightAuto(LinearOpMode opMode) {
        super(opMode);
    }
    Pose2d startPose = new Pose2d(new Vector2d(-24, -72), 90);

    @Override
    public void init() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void stop() {

    }

    @Override
    public Pose2d getStartPose() {
        return startPose;
    }
}
