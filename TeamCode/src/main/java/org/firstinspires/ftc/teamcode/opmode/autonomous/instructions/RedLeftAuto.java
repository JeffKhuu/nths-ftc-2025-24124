package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.MotorWrist;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewMotorWrist;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 1, cycles = 0)
public class RedLeftAuto extends AutoInstructions {

    // Instantiate subsystems
    public static Pose2d startPose = new Pose2d(new Vector2d(-12, -64), Math.toRadians(90));

//    SequentialAction depositSample = new SequentialAction(
//            driveTrain.strafeTo(-57, -56),  // Deposit Sample
//            driveTrain.turnTo(225),
//            slides.moveTo(Slide.SlideState.HIGH_BUCKET.position),
//            driveTrain.strafeTo(-59, -59),
//            claw.moveTo(Claw.ClawState.OPEN),
//            new SleepAction(1)
//    );

    RedLeftAuto(LinearOpMode opMode) {
        super(opMode, startPose);
    }

    @Override
    public void init() {
        driveTrain.mecanumDrive.pose = startPose;
        opMode.telemetry.addLine("Ready");
        opMode.telemetry.update();
    }

    @Override
    public void execute() {

        Actions.runBlocking(new SequentialAction(
                //claw.moveTo(Claw.ClawState.CLOSED),

                // Hang Preloaded Specimen
                wrist.moveTo(NewMotorWrist.WristState.INACTIVE.position),
                slides.moveTo(Slide.SlideState.HIGH_BUCKET.position),
                new SleepAction(5)
        ));
    }

    @Override
    public void stop() {
        FieldConstants.savePose(driveTrain.mecanumDrive.pose);
    }
}
