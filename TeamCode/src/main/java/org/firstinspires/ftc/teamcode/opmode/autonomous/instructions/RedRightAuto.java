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
                driveTrain.turnTo(90),
                driveTrain.strafeTo(8, -35),
                new SleepAction(0.2),
                slides.moveTo(Slide.SlideState.CLIP_HANG.position),
                driveTrain.strafeTo(8, -50)
        ));
    }

    @Override
    public void stop() {

    }
}
