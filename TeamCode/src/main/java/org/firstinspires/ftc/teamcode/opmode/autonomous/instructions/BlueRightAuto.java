package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 0, cycles = 0)
public class BlueRightAuto extends AutoInstructions{
    private static final Pose2d startPose = new Pose2d(new Vector2d(-0, 0), 0);
    BlueRightAuto(LinearOpMode opMode) {
        super(opMode, startPose);
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void stop() {

    }
}
