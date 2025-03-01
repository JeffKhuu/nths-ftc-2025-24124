package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.subsystems.NewMotorWrist.WristState;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutoInstructions;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 0, cycles = 0)
public class BlueRightAuto extends AutoInstructions {
    public static Pose2d startPose = new Pose2d(new Vector2d(12, -64), Math.toRadians(90));

    public BlueRightAuto(LinearOpMode opMode) {
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
                new ParallelAction(
                        wrist.moveTo(WristState.HOME.position),
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position),
                        driveTrain.strafeTo(12, -37)
                ),
                driveTrain.strafeTo(12, -34),
                slides.moveTo(5000),
                driveTrain.strafeTo(34, -45),

                //1st Sample Push (untested)
                driveTrain.strafeTo(48, -28, 0),
                driveTrain.strafeTo(58, -65),


                //2nd Specimen Clip
                // Get Specimen from Observation Station
                new ParallelAction(
                        driveTrain.strafeTo(60, -65),
                        slides.moveTo(Slide.SlideState.CLIPPER.position)
                ),
                new SleepAction(0.5),
                driveTrain.strafeTo(65, -65),
                slides.moveTo(2650),
//
                // Hang Specimen using Static Clip
                new ParallelAction(
                        wrist.moveTo(WristState.HOME.position),
                        driveTrain.strafeTo(8, -37, 90),
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position)
                ),
                driveTrain.strafeTo(8, -34),
                slides.moveTo(5000),

                // Get specimen from Observation Station
                new ParallelAction(
                        driveTrain.strafeTo(60, -65, 0),
                        slides.moveTo(Slide.SlideState.CLIPPER.position)
                ),
                new SleepAction(0.5),
                driveTrain.strafeTo(65, -65),
                slides.moveTo(2650),

                // Hang Specimen using Static Clip
                new ParallelAction(
                        wrist.moveTo(WristState.HOME.position),
                        driveTrain.strafeTo(4, -37, 90),
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position)
                ),
                driveTrain.strafeTo(4, -34),
                slides.moveTo(5000),

                // Park
                new ParallelAction(
                        driveTrain.strafeTo(48, -60),
                        wrist.moveTo(WristState.HOME.position),
                        slides.moveTo(Slide.SlideState.HOME.position)
                )
        ));
    }

    @Override
    public void stop() {

    }
}
