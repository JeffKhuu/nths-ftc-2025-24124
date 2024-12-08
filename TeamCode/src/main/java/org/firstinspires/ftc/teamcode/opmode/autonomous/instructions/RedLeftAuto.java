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
                claw.moveTo(Claw.ClawState.CLOSED),
                wrist.moveTo(Wrist.WristState.INACTIVE),

                // Hang Preloaded Specimen
                new ParallelAction(
                    slides.moveTo(Slide.SlideState.HIGH_RUNG.position),
                        driveTrain.strafeTo(-12, -38)
                ),
                wrist.moveTo(Wrist.WristState.HANG),
                new SleepAction(0.2),
                slides.moveTo(Slide.SlideState.HANG.position),
                claw.moveTo(Claw.ClawState.OPEN),
                new SleepAction(0.2),

                // Pick Up Sample
                new ParallelAction(
                        driveTrain.strafeTo(-50, -41),
                        slides.moveTo(Slide.SlideState.ACTIVE.position)
                ),
                claw.moveTo(Claw.ClawState.OPEN),
                new SleepAction(0.2),
                wrist.moveTo(Wrist.WristState.ACTIVE),
                new SleepAction(0.4),
                claw.moveTo(Claw.ClawState.CLOSED),
                new SleepAction(0.2),
                wrist.moveTo(Wrist.WristState.INACTIVE),

                 // Deposit Sample
                driveTrain.turnTo(225),
                new ParallelAction(
                        driveTrain.strafeTo(-57, -56),
                        slides.moveTo(Slide.SlideState.HIGH_BUCKET.position)),
                driveTrain.strafeTo(-59.2, -59.4),
                claw.moveTo(Claw.ClawState.OPEN),
                new SleepAction(0.2),
                driveTrain.strafeTo(-57, -57), // FIXME REMOVE THIS IF AUTO STOPS WORKING
                new SleepAction(0.2),

                // Pick Up Sample
                driveTrain.turnTo(90),
                new SleepAction(0.2),
                new ParallelAction(
                        driveTrain.strafeTo(-60, -40),
                        slides.moveTo(Slide.SlideState.ACTIVE.position)),
                claw.moveTo(Claw.ClawState.OPEN),
                new SleepAction(0.2),
                wrist.moveTo(Wrist.WristState.ACTIVE),
                new SleepAction(0.4),
                claw.moveTo(Claw.ClawState.CLOSED),
                new SleepAction(0.2),
                wrist.moveTo(Wrist.WristState.INACTIVE),

                // Deposit Sample
                driveTrain.turnTo(225),
                new ParallelAction(
                        driveTrain.strafeTo(-60, -56),
                        slides.moveTo(Slide.SlideState.HIGH_BUCKET.position)),
                driveTrain.strafeTo(-59.5, -59.5),
                claw.moveTo(Claw.ClawState.OPEN),
                new SleepAction(0.2),

                // Park in the ascent zone
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.HOME.position),
                        new SequentialAction(
                                new ParallelAction(
                                        driveTrain.turnTo(0),
                                        wrist.moveTo(Wrist.WristState.HOME),
                                        claw.moveTo(Claw.ClawState.CLOSED)
                                ),
                                driveTrain.strafeTo(-40, -15)
                        )
                )



        ));
    }

    @Override
    public void stop() {
        FieldConstants.savePose(driveTrain.mecanumDrive.pose);
    }
}
