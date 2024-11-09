package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 1, cycles = 0)
public class RedLeftAuto extends AutoInstructions {

    // Instantiate subsystems
    public static Pose2d startPose = new Pose2d(new Vector2d(-12, -64), Math.toRadians(90));

    RedLeftAuto(LinearOpMode opMode) {
        super(opMode, startPose);
    }

    @Override
    public void init() {
        driveTrain.mecanumDrive.pose = startPose;
        opMode.telemetry.addData("Pose", driveTrain.mecanumDrive.pose);
        opMode.telemetry.update();
    }

    @Override
    public void execute() {

        Actions.runBlocking(new SequentialAction(
                claw.setTo(Claw.ClawState.CLOSED),
                wrist.moveTo(Wrist.WristState.INACTIVE),

                slides.moveTo(Slide.SlideState.HIGH_RUNG.position), // Hang Preloaded Specimen
                driveTrain.strafeTo(-12, -36),
                new SleepAction(1),
                slides.moveTo(Slide.SlideState.HIGH_RUNG_HANG.position),
                claw.setTo(Claw.ClawState.OPEN),

                new SleepAction(1),
                slides.moveTo(Slide.SlideState.LOW_RUNG.position),
                wrist.moveTo(Wrist.WristState.HOME),
                claw.setTo(Claw.ClawState.CLOSED),

                driveTrain.strafeTo(-48, -36), // Pick Up Sample
                wrist.moveTo(Wrist.WristState.ACTIVE),
                claw.setTo(Claw.ClawState.OPEN),
                new SleepAction(1),
                claw.setTo(Claw.ClawState.CLOSED),

                driveTrain.turnTo(180), // Deposit Sample
                //driveTrain.strafeTo(-60, -64),
                slides.moveTo(Slide.SlideState.HIGH_BUCKET.position),
                claw.setTo(Claw.ClawState.OPEN)

        ));
    }

    @Override
    public void stop() {
        FieldConstants.savePose(driveTrain.mecanumDrive.pose);
    }
}
