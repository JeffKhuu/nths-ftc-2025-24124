package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewMotorWrist.WristState;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutoInstructions;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 1, cycles = 0)
public class RedLeftAuto extends AutoInstructions {

    // Instantiate subsystems
    public static Pose2d startPose = new Pose2d(new Vector2d(-38, -64), Math.toRadians(180));

//    SequentialAction depositSample = new SequentialAction(
//            driveTrain.strafeTo(-57, -56),  // Deposit Sample
//            driveTrain.turnTo(225),
//            slides.moveTo(Slide.SlideState.HIGH_BUCKET.position),
//            driveTrain.strafeTo(-59, -59),
//            claw.moveTo(Claw.ClawState.OPEN),
//            new SleepAction(1)
//    );

    public RedLeftAuto(LinearOpMode opMode) {
        super(opMode, startPose);
    }

    @Override
    public void init() {
        driveTrain.mecanumDrive.pose = startPose;
        opMode.telemetry.addLine("Ready");
        opMode.telemetry.update();

        intake.claw.setPosition(Claw.ClawState.CLOSED.position);
        driveTrain.savePose(startPose);
    }

    @Override
    public void execute() {

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveTrain.strafeTo(-51, -55, 225),
                                slides.moveTo(Slide.SlideState.HIGH_BUCKET.position),
                                wrist.moveTo(WristState.INACTIVE.position)
                        ),
                        wrist.moveTo(WristState.INACTIVE.position - 100),
                        intake.moveTo(Claw.ClawState.OPEN),
                        new SleepAction(0.5),
                        wrist.moveTo(WristState.INACTIVE.position),

                        new ParallelAction(
                                strafeToLinearHeading(-46, -42, 90),
                                slides.moveTo(300)
                        ),
                        wrist.moveTo(WristState.ACTIVE.position),
                        intake.moveTo(Claw.ClawState.CLOSED),
                        new SleepAction(0.5),

                        new ParallelAction(
                                driveTrain.strafeTo(-51, -55, 225),
                                slides.moveTo(Slide.SlideState.HIGH_BUCKET.position),
                                wrist.moveTo(WristState.INACTIVE.position)
                        ),
                        wrist.moveTo(WristState.INACTIVE.position - 100),
                        intake.moveTo(Claw.ClawState.OPEN),
                        new SleepAction(0.5),
                        wrist.moveTo(WristState.INACTIVE.position)



//                        new ParallelAction(
//                                driveTrain.strafeTo(-52, -56, 225),
//                                slides.moveTo(Slide.SlideState.HIGH_BUCKET.position),
//                                wrist.moveTo(WristState.INACTIVE.position)
//                        ),
//                        intake.moveTo(Claw.ClawState.OPEN)
                )




//                driveTrain.mecanumDrive.actionBuilder(startPose)
//                        .strafeToLinearHeading(new Vector2d(-52, -56), Math.toRadians(225))
//                        .waitSeconds(4)
//
//                        .strafeToLinearHeading(new Vector2d(-47, -40), Math.toRadians(90))
//                        .waitSeconds(1)
//                        .strafeToLinearHeading(new Vector2d(-52, -56), Math.toRadians(225))
//                        .waitSeconds(4)
//
//                        .strafeToLinearHeading(new Vector2d(-58, -40), Math.toRadians(90))
//                        .waitSeconds(1)
//                        .strafeToLinearHeading(new Vector2d(-52, -56), Math.toRadians(225))
//                        .waitSeconds(4)
//
//                        .strafeToLinearHeading(new Vector2d(-53, -25), Math.toRadians(180))
//                        .waitSeconds(1)
//                        .strafeToLinearHeading(new Vector2d(-52, -56), Math.toRadians(225))
//                        .waitSeconds(4)
//                        .strafeTo(new Vector2d(-52, -56))
//                        .build()
        );
    }

    @Override
    public void stop() {

    }

    private Action strafeToLinearHeading(int x, int y, int heading){
        driveTrain.savePose(new Pose2d(new Vector2d(x, y), Math.toRadians(heading)));
        return driveTrain.mecanumDrive.actionBuilder(driveTrain.getLastSavedPose())
                .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading))
                .build();
    }
}
