package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.MotorWrist;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 0, cycles = 0)
public class BlueRightAuto extends AutoInstructions {
    public static Pose2d startPose = new Pose2d(new Vector2d(12, -64), Math.toRadians(90));

    BlueRightAuto(LinearOpMode opMode) {
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
                        wrist.moveTo(-100),
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position),
                        driveTrain.strafeTo(12, -37)
                ),
                driveTrain.strafeTo(12, -34),
                slides.moveTo(Slide.SlideState.CLIP_HANG.position),
                driveTrain.strafeTo(34, -50),

                // TODO: Push all 3 specimen into the observation station
                //Push 3 specimen into the observation station

                //1st Sample Push (untested)
                driveTrain.strafeTo(40,-20),
                driveTrain.turnTo(90),

                //2nd Specimen Clip
                // Get Specimen from Observation Station
                driveTrain.strafeTo(48, -50),
                new ParallelAction(
                        //wrist.moveTo(-170),
                        driveTrain.turnTo(270),
                        slides.moveTo(Slide.SlideState.CLIPPER.position+300)
                ),
                driveTrain.strafeTo(48, -70),
                slides.moveTo(Slide.SlideState.HANG.position),

                // Hang Specimen using Static Clip
                new ParallelAction(
                        wrist.moveTo(-100),
                        driveTrain.strafeTo(8, -50)

                ),
                driveTrain.turnTo(90),
                slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position + 1000),

                new SleepAction(0.2),
                driveTrain.strafeTo(8, -32),
                slides.moveTo(Slide.SlideState.CLIP_HANG.position),
                driveTrain.strafeTo(8, -50),

                //2nd Specimen Clip
                // Get Specimen from Observation Station
                driveTrain.strafeTo(48, -50),
                new ParallelAction(
                        //wrist.moveTo(-170),
                        driveTrain.turnTo(270),
                        slides.moveTo(Slide.SlideState.CLIPPER.position+300)
                ),
                driveTrain.strafeTo(48, -70),
                slides.moveTo(Slide.SlideState.HANG.position),

                // Hang Specimen using Static Clip
                new ParallelAction(
                        wrist.moveTo(-100),
                        driveTrain.strafeTo(4, -50)

                ),
                driveTrain.turnTo(90),
                slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position + 1000),

                new SleepAction(0.2),
                driveTrain.strafeTo(4, -32),
                slides.moveTo(Slide.SlideState.CLIP_HANG.position),
                driveTrain.strafeTo(4, -50)
        ));
    }

    @Override
    public void stop() {

    }
}
