package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewMotorWrist.WristState;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PushMechanism;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutoInstructions;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

import java.lang.Math;

@AutonomousEx(preload = 1, cycles = 3)
public class BlueLeftAuto extends AutoInstructions {

    // Instantiate subsystems
    public static Pose2d startPose = new Pose2d(new Vector2d(12, -64), Math.toRadians(90));
    TranslationalVelConstraint highSpeed = new TranslationalVelConstraint(70);
    ProfileAccelConstraint highAcc = new ProfileAccelConstraint(-30, 70);

    public BlueLeftAuto(LinearOpMode opMode) {
        super(opMode, startPose);
    }

    @Override
    public void init() {
        driveTrain.mecanumDrive.pose = startPose;
        opMode.telemetry.addLine("Ready");
        opMode.telemetry.update();
        driveTrain.savePose(startPose);
    }

    @Override
    public void execute() {
        driveTrain.mecanumDrive.pose = startPose;
        Actions.runBlocking(new SequentialAction(
                        // Hang Preloaded Specimen
                        new ParallelAction(
                                wrist.moveTo(WristState.HOME.position - 50),
                                slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position-400),
                                driveTrain.strafeTo(3, -37)
                        ),
                        driveTrain.strafeTo(3, -33),

                        new ParallelAction(
                                slides.moveTo(5000),
                                new SequentialAction(
                                        new SleepAction(0.15),
                                        driveTrain.strafeTo(3, -43)
                                )
                        ),

                        // Push 2 Samples into Observation Station
                        driveTrain.strafeTo(34, -33, 75),
                        pusher.moveTo(PushMechanism.PushState.ACTIVE),
                        new SleepAction(0.05),
                        driveTrain.strafeTo(44, -53, 0),

                        new ParallelAction(
                                slides.moveTo(Slide.SlideState.CLIPPER.position),
                                new SequentialAction(
                                        //Push Second Sample
                                        pusher.moveTo(PushMechanism.PushState.INACTIVE),
                                        // Push Forawrd
                                        driveTrain.strafeTo(42, -30, 75),
                                        pusher.moveTo(PushMechanism.PushState.ACTIVE),
                                        new SleepAction(0.05),

                                        driveTrain.strafeTo(54, -63, 0),
                                        pusher.moveTo(PushMechanism.PushState.INACTIVE),

                                        // Retrieve Specimen from Observation Station
                                        driveTrain.splineToHeading(60, -59, Math.toRadians(0), Math.toRadians(0)),
                                        driveTrain.splineTo(63, -59, Math.toRadians(0))
                                )
                        ),


                        // Deposit Specimen on High Chamber
                        new ParallelAction(
                                slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position-200),
                                driveTrain.strafeToSplineHeading(5, -35, 90)

                        ),
                        slides.moveTo(5500),
//                new ParallelAction(
//
//                        new SequentialAction(
//                                new SleepAction(0.15),
//                                driveTrain.strafeTo(10, -48, new TranslationalVelConstraint(70), new ProfileAccelConstraint(-30, 70))
//                        )
//                ),


                        // Retrieve Specimen from Observation Station
                        new ParallelAction(
                                slides.moveTo(Slide.SlideState.CLIPPER.position),
                                new SequentialAction(
                                        driveTrain.strafeToSplineHeading(40, -65, 271)
                                )
                        ),

                        // Deposit Specimen on High Chamber
                        new ParallelAction(
                                driveTrain.strafeToSplineHeading(6, -35, 90),
                                slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position-200)
                        ),
                        slides.moveTo(5500),
//                new ParallelAction(
//
//                        new SequentialAction(
//                                new SleepAction(0.15),
//                                driveTrain.strafeTo(10, -48, new TranslationalVelConstraint(70), new ProfileAccelConstraint(-30, 70))
//                        )
//                ),

                        // Retrieve Specimen from Observation Station
                        new ParallelAction(
                                slides.moveTo(Slide.SlideState.CLIPPER.position),
                                driveTrain.strafeToSplineHeading(40, -65, 271)
                        ),

                        // Deposit Specimen on High Chamber
                        new ParallelAction(
                                //driveTrain.strafeToSplineHeading(5, -33, 90), // keep old?
                                slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position-200),
                                driveTrain.mecanumDrive.actionBuilder(new Pose2d(40, -65, Math.toRadians(270)))
                                        .strafeToSplineHeading(new Vector2d(7, -36), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(7, -33), Math.toRadians(90))
                                        .build()
                        ),
                        new ParallelAction(
                                slides.moveTo(Slide.SlideState.HOME.position),
                                new SequentialAction(
                                        new SleepAction(0.15),
                                        driveTrain.strafeTo(40, -62, new TranslationalVelConstraint(70), new ProfileAccelConstraint(-30, 70))
                                )
                        ))


        );
    }

    @Override
    public void stop() {

    }
}