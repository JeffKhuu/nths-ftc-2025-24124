package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.MotorWrist;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewMotorWrist;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PushMechanism;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 0, cycles = 0)
public class RedRightAuto extends AutoInstructions {
    public static Pose2d startPose = new Pose2d(new Vector2d(12, -64), Math.toRadians(90));

    ProfileAccelConstraint highAcc = new ProfileAccelConstraint(-30, 70);
    VelConstraint highSpeed = new TranslationalVelConstraint(70);

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
                // Hang Preloaded Specimen
                new ParallelAction(
                        wrist.moveTo(NewMotorWrist.WristState.HOME.position-50),
                        slides.moveTo(Slide.SlideState.HIGH_RUNG.position),
                        driveTrain.strafeTo(4, -37, highAcc, highSpeed)
                ),
                driveTrain.strafeTo(4, -34, highAcc, highSpeed),
                slides.moveTo(Slide.SlideState.CLIPPER.position),
                driveTrain.strafeTo(4, -40),

                // Push 2 Samples into Observation Station
                driveTrain.strafeTo(34, -33, 75),
                pusher.moveTo(PushMechanism.PushState.ACTIVE),
                new SleepAction(0.1),
                driveTrain.strafeTo(44, -60, 0),

//                pusher.moveTo(PushMechanism.PushState.INACTIVE),
//                driveTrain.strafeTo(50, -24, 45),
//                pusher.moveTo(PushMechanism.PushState.ACTIVE),
//                new SleepAction(0.1),
//                driveTrain.strafeTo(64, -60, 0),
//                pusher.moveTo(PushMechanism.PushState.INACTIVE),


                new ParallelAction(
                        slides.moveTo(Slide.SlideState.CLIPPER.position),
                        new SequentialAction(
                                //Push Second Sample
                                pusher.moveTo(PushMechanism.PushState.INACTIVE),
                                driveTrain.strafeTo(42, -30, 75),
                                pusher.moveTo(PushMechanism.PushState.ACTIVE),
                                new SleepAction(0.1),
                                driveTrain.strafeTo(54, -63, 0),
                                pusher.moveTo(PushMechanism.PushState.INACTIVE),

                                // Retrieve Specimen from Observation Station
                                driveTrain.strafeTo(60, -60),
                                driveTrain.strafeTo(63, -60),
                                new SleepAction(0.1)
                        )
                ),


                // Deposit Specimen on High Chamber
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position),
                        driveTrain.strafeTo(6, -36, 90)
                ),
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.CLIPPER.position),
                        driveTrain.strafeTo(6, -34)
                ),

                
                // Retrieve Specimen from Observation Station
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.CLIPPER.position),
                        new SequentialAction(
                                driveTrain.strafeTo(55, -60, 0),
                                driveTrain.strafeTo(63, -60),
                                new SleepAction(0.1)
                        )
                ),

                // Deposit Specimen on High Chamber
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position),
                        driveTrain.strafeTo(8, -36, 90)
                ),
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.CLIPPER.position),
                        driveTrain.strafeTo(8, -34)
                ),


                // Retrieve Specimen from Observation Station
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.CLIPPER.position),
                        new SequentialAction(
                                driveTrain.strafeTo(55, -60, 0),
                                driveTrain.strafeTo(63, -60),
                                new SleepAction(0.1)
                        )
                ),

                // Deposit Specimen on High Chamber
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position),
                        driveTrain.strafeTo(10, -36, 90)
                ),
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.CLIPPER.position),
                        driveTrain.strafeTo(10, -34)
                )



        ));
    }

    @Override
    public void stop() {

    }
}
