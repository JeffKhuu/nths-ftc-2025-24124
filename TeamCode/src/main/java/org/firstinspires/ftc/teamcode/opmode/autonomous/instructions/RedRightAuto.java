package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 0, cycles = 0)
public class RedRightAuto extends AutoInstructions {
    private static final Pose2d startPose = new Pose2d(new Vector2d(-0, 0), 0);
    RedRightAuto(LinearOpMode opMode) {
        super(opMode, startPose);
    }


    @Override
    public void init() {
        FieldConstants.savePose(startPose);
    }

    @Override
    public void execute() {
        Actions.runBlocking(new SequentialAction(
                driveTrain.strafeTo(-24, 0),
                new SleepAction(1),
                driveTrain.strafeTo(-48, 0),
                new SleepAction(1),
                driveTrain.turnTo(90),
                driveTrain.strafeTo(-48, 24)
        ));
    }

    @Override
    public void stop() {

    }
}
