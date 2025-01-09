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

                // Hang Preloaded Specimen
                new ParallelAction(
                        wrist.moveTo(-170),
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position),
                        driveTrain.strafeTo(-12, -37)
                ),
                driveTrain.strafeTo(-12, -34),
                slides.moveTo(Slide.SlideState.CLIP_HANG.position),
                driveTrain.strafeTo(-34, -50),

                // Pick Up Sample
                new ParallelAction(
                        driveTrain.strafeTo(-50, -41),
                        slides.moveTo(Slide.SlideState.HOME.position)
                ),
                claw.moveTo(Claw.ClawState.OPEN),
                new SleepAction(0.2),
                wrist.moveTo(MotorWrist.WristState.ACTIVE.position),
                claw.moveTo(Claw.ClawState.CLOSED),
                new SleepAction(0.2),
                wrist.moveTo(MotorWrist.WristState.INACTIVE.position),

                 // Deposit Sample
                driveTrain.turnTo(225),
                new ParallelAction(
                        driveTrain.strafeTo(-53, -53),
                        slides.moveTo(Slide.SlideState.HIGH_BUCKET.position),
                        wrist.moveTo(MotorWrist.WristState.INACTIVE.position-50)
                ),
                driveTrain.strafeTo(-61, -61),

                claw.moveTo(Claw.ClawState.OPEN),
                new SleepAction(0.2),
                wrist.moveTo(MotorWrist.WristState.INACTIVE.position+50),
                new SleepAction(0.2),

                // Pick Up Sample
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.HOME.position),
                        new SequentialAction(
                                driveTrain.strafeTo(-57, -57),
                                new SleepAction(0.2),
                                driveTrain.turnTo(90),
                                new SleepAction(0.2),
                                driveTrain.strafeTo(-60, -40)
                        )
                ),

                wrist.moveTo(MotorWrist.WristState.ACTIVE.position),
                claw.moveTo(Claw.ClawState.OPEN),
                new SleepAction(0.2),
                claw.moveTo(Claw.ClawState.CLOSED),
                new SleepAction(0.2),
                wrist.moveTo(MotorWrist.WristState.INACTIVE.position),

                // Deposit Sample
                driveTrain.turnTo(225),
                new ParallelAction(
                        driveTrain.strafeTo(-53, -53),
                        slides.moveTo(Slide.SlideState.HIGH_BUCKET.position),
                        wrist.moveTo(MotorWrist.WristState.INACTIVE.position-50)
                ),
                driveTrain.strafeTo(-61, -61),

                claw.moveTo(Claw.ClawState.OPEN),
                new SleepAction(0.2),
                wrist.moveTo(MotorWrist.WristState.INACTIVE.position+50),
                new SleepAction(0.2),


                // Reset for TeleOp
                new ParallelAction(
                        driveTrain.turnTo(0),
                        slides.moveTo(Slide.SlideState.HOME.position),
                        wrist.moveTo(MotorWrist.WristState.HOME.position),
                        claw.moveTo(Claw.ClawState.CLOSED)
                )
        ));
    }

    @Override
    public void stop() {
        FieldConstants.savePose(driveTrain.mecanumDrive.pose);
    }
}
