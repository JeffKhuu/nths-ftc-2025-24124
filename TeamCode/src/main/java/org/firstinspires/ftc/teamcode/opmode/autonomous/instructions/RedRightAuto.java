package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 0, cycles = 0)
public class RedRightAuto extends AutoInstructions {
    public static Pose2d startPose = new Pose2d(new Vector2d(12, -64), Math.toRadians(90));

    RedRightAuto(LinearOpMode opMode) {
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
                claw.setTo(Claw.ClawState.CLOSED),
                wrist.moveTo(Wrist.WristState.INACTIVE),
                //1st Specimen Clip
                // Hang Preloaded Specimen
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.HIGH_RUNG.position),
                        driveTrain.strafeTo(12, -40)
                ),
                wrist.moveTo(Wrist.WristState.HANG),
                new SleepAction(0.2),
                slides.moveTo(Slide.SlideState.HIGH_RUNG.position),
                claw.setTo(Claw.ClawState.OPEN),
                new SleepAction(0.2),
                wrist.moveTo(Wrist.WristState.HOME),
                driveTrain.strafeTo(12, -50),

                // TODO: Push all 3 specimen into the observation station
                //2nd Specimen Clip
                // Get Specimen from Observation Station
                driveTrain.strafeTo(48, -50),
                new ParallelAction(
                        driveTrain.turnTo(270),
                        slides.moveTo(Slide.SlideState.CLIPPER.position)
                ),
                driveTrain.strafeTo(48, -70),

                // Hang Specimen using Static Clip
                new ParallelAction(
                        driveTrain.strafeTo(8, -50),
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position)
                ),
                driveTrain.strafeTo(8, -35),
                driveTrain.turnTo(90),
                new SleepAction(0.2),
                slides.moveTo(Slide.SlideState.CLIP_HANG.position),
                driveTrain.strafeTo(8, -50),

                //Push 3 specimen into the observation station

                //1st Sample Push (untested)
                driveTrain.strafeTo(40,-36),
                driveTrain.strafeTo(44,-36),
                driveTrain.strafeTo(44,-50),
                driveTrain.strafeTo(44,-60),

                //2nd Sample Push (untested)

                driveTrain.strafeTo(44,-36),
                driveTrain.strafeTo(54,-36),
                driveTrain.strafeTo(54,-50),
                driveTrain.strafeTo(54,-65),

                //3rd Sample Push (untested) (needs precis co-ordinates)

                driveTrain.strafeTo(54,-36),
                driveTrain.turnTo(0),
                driveTrain.strafeTo(64,-50),
                driveTrain.strafeTo(64,-60),
                driveTrain.strafeTo(64,-65),
                driveTrain.strafeTo(54,-65),

                //3rd Specimen Clip

                driveTrain.strafeTo(48, -50),
                new ParallelAction(
                        driveTrain.turnTo(270),
                        slides.moveTo(Slide.SlideState.CLIPPER.position)
                ),
                driveTrain.strafeTo(48, -70),

                // Hang Specimen using Static Clip

                new ParallelAction(
                        driveTrain.strafeTo(4, -50),
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position)
                ),
                driveTrain.turnTo(90),
                driveTrain.strafeTo(4, -35),
                new SleepAction(0.2),
                slides.moveTo(Slide.SlideState.CLIP_HANG.position),
                driveTrain.strafeTo(4, -50),

                //4th Specimen Clip


                driveTrain.strafeTo(48, -50),
                new ParallelAction(
                        driveTrain.turnTo(270),
                        slides.moveTo(Slide.SlideState.CLIPPER.position)
                ),
                driveTrain.strafeTo(48, -70),

                // Hang Specimen using Static Clip

                new ParallelAction(
                        driveTrain.strafeTo(6, -50),
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position)
                ),
                driveTrain.turnTo(90),
                driveTrain.strafeTo(6, -35),
                new SleepAction(0.2),
                slides.moveTo(Slide.SlideState.CLIP_HANG.position),
                driveTrain.strafeTo(6, -50),

                //5th Specimen Clip

                driveTrain.strafeTo(48, -50),
                new ParallelAction(
                        driveTrain.turnTo(270),
                        slides.moveTo(Slide.SlideState.CLIPPER.position)
                ),
                driveTrain.strafeTo(48, -70),

                // Hang Specimen using Static Clip

                new ParallelAction(
                        driveTrain.strafeTo(11, -50),
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position)
                ),
                driveTrain.turnTo(90),
                driveTrain.strafeTo(11, -35),
                new SleepAction(0.2),
                slides.moveTo(Slide.SlideState.CLIP_HANG.position),
                driveTrain.strafeTo(11, -50)

        ));
    }

    @Override
    public void stop() {

    }
}
