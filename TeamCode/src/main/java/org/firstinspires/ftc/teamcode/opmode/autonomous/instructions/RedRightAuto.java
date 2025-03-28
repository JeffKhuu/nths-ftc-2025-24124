package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.InstantAction;
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
public class RedRightAuto extends AutoInstructions {

    // Instantiate subsystems
    public static Pose2d startPose = new Pose2d(new Vector2d(12, -64), Math.toRadians(90));
    TranslationalVelConstraint highSpeed = new TranslationalVelConstraint(70);
    ProfileAccelConstraint highAcc = new ProfileAccelConstraint(-30, 70);

    public RedRightAuto(LinearOpMode opMode) {
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
                        driveTrain.strafeTo(3, -32),

                        new ParallelAction(
                                slides.moveTo(4500),
                                new SequentialAction(
                                        new SleepAction(0.15),
                                        driveTrain.strafeTo(3, -45)
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
                                        driveTrain.splineTo(64, -59, Math.toRadians(0))
                                )
                        ),


                        // Deposit Specimen on High Chamber
                        new ParallelAction(
                                slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position-200),
                                driveTrain.strafeToSplineHeading(5, -34, 90)

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
                                        driveTrain.strafeToSplineHeading(40, -66, 271)
                                )
                        ),

                        // Deposit Specimen on High Chamber
                        new ParallelAction(
                                driveTrain.strafeToSplineHeading(6, -34, 90),
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
                                driveTrain.strafeToSplineHeading(40, -66, 271)
                        ),

                        // Deposit Specimen on High Chamber
                        new ParallelAction(
                                //driveTrain.strafeToSplineHeading(5, -33, 90), // keep old?
                                slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position-200),
                                driveTrain.strafeToSplineHeading(6.5, -34, 90)
                        ),
                        new ParallelAction(
                                slides.moveTo(Slide.SlideState.HOME.position),
                                new SequentialAction(
                                        new SleepAction(0.50),
                                        driveTrain.strafeTo(10, -48, new TranslationalVelConstraint(70), new ProfileAccelConstraint(-30, 70))
                                )
                        ))


        );
    }

    @Override
    public void stop() {

    }
}