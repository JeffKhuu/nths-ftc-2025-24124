package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.subsystems.NewMotorWrist;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PushMechanism;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutoInstructions;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 1, cycles = 3)
public class PrototypeAuto extends AutoInstructions {
    private static final Pose2d startPose = new Pose2d(new Vector2d(8, -64), Math.toRadians(90));

    public PrototypeAuto(LinearOpMode opMode) {
        super(opMode, startPose);
    }

    @Override
    public void init() {
        driveTrain.mecanumDrive.updatePoseEstimate();
    }

    @Override
    public void execute() {
        Actions.runBlocking(
                new SequentialAction(
                        hangPreload(),
                        pushSamples()
//                        hangSpecimen(6),
//                        retrieveSpecimen(),
//                        hangSpecimen(8),
//                        retrieveSpecimen(),
//                        hangSpecimen(9)
                )
        );
    }

    @Override
    public void stop() {

    }

    Action hangPreload(){
        return driveTrain.mecanumDrive.actionBuilder(PrototypeAuto.startPose)
                .afterTime(0.1, wrist.moveTo(-50))
                .afterTime(0.1, slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position))
                .splineToConstantHeading(new Vector2d(4, -37), Math.toRadians(90))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(4, -32), Math.toRadians(90))
                .stopAndAdd(slides.moveTo(5000))
                .splineToConstantHeading(new Vector2d(4, -34), Math.toRadians(90))
                .stopAndAdd(driveTrain.mecanumDrive::updatePoseEstimate)
                .build();
    }

    Action pushSamples(){
        Pose2d pose = driveTrain.mecanumDrive.pose;
        return driveTrain.mecanumDrive.actionBuilder(pose)
                .splineToSplineHeading(new Pose2d(new Vector2d(30, -25), Math.toRadians(75)), Math.toRadians(90))
                .stopAndAdd(pusher.moveTo(PushMechanism.PushState.ACTIVE))
                .splineToSplineHeading(new Pose2d(new Vector2d(38, -60), Math.toRadians(0)), Math.toRadians(90))
                .stopAndAdd(pusher.moveTo(PushMechanism.PushState.INACTIVE))

                .splineToSplineHeading(new Pose2d(new Vector2d(36, -25), Math.toRadians(75)), Math.toRadians(90))
                .stopAndAdd(pusher.moveTo(PushMechanism.PushState.ACTIVE))
                .splineToSplineHeading(new Pose2d(new Vector2d(42, -60), Math.toRadians(0)), Math.toRadians(90))
                .stopAndAdd(pusher.moveTo(PushMechanism.PushState.INACTIVE))
                .splineToConstantHeading(new Vector2d(63, -58), Math.toRadians(0))
                .stopAndAdd(driveTrain.mecanumDrive::updatePoseEstimate)
                .build();
    }

    Action hangSpecimen(int position){
        Pose2d pose = driveTrain.mecanumDrive.pose;

        return driveTrain.mecanumDrive.actionBuilder(pose)
                .afterTime(0.1, slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(new Vector2d(position, -36), Math.toRadians(90)), Math.toRadians(90))
                .splineTo(new Vector2d(position, -34), Math.toRadians(90))
                .stopAndAdd(slides.moveTo(Slide.SlideState.CLIPPER.position))
                .splineToConstantHeading(new Vector2d(position, -38), Math.toRadians(90))
                .stopAndAdd(driveTrain.mecanumDrive::updatePoseEstimate)
                .build();
    }

    Action retrieveSpecimen(){
        Pose2d pose = driveTrain.mecanumDrive.pose;

        return driveTrain.mecanumDrive.actionBuilder(pose)
                .afterTime(0.1, slides.moveTo(Slide.SlideState.CLIPPER.position))
                .splineToLinearHeading(new Pose2d(new Vector2d(40, -60), Math.toRadians(270)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(41, -66), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(40, -55), Math.toRadians(270))
                .stopAndAdd(driveTrain.mecanumDrive::updatePoseEstimate)
                .build();
    }
}
