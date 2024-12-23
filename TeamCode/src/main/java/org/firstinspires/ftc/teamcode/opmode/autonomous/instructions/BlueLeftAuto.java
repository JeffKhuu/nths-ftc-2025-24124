package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

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

@AutonomousEx(preload = 1, cycles = 0)
public class BlueLeftAuto extends AutoInstructions {
    private static final Pose2d startPose = new Pose2d(new Vector2d(-0, 0), 0);
    BlueLeftAuto(LinearOpMode opMode) {
        super(opMode, startPose);
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
        Actions.runBlocking(new SequentialAction(
                // driveTrain.moveTo(x, y),
                claw.moveTo(Claw.ClawState.CLOSED),
                wrist.moveTo(Wrist.WristState.INACTIVE),
                slides.moveTo(Slide.SlideState.HIGH_RUNG.position),
                new SleepAction(1),
                slides.moveTo(Slide.SlideState.HOVER.position),
                claw.moveTo(Claw.ClawState.OPEN),
                new SleepAction(1),
                // driveTrain.moveTo(observation station),
                slides.moveTo(Slide.SlideState.HOME.position),
                claw.moveTo(Claw.ClawState.CLOSED),
                new SleepAction(1)

        ));
    }

    @Override
    public void stop() {}
}
