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

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewMotorWrist.WristState;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PushMechanism;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutoInstructions;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 1, cycles = 0)
public class BlueLeftAuto extends AutoInstructions {

    // Instantiate subsystems
    public static Pose2d startPose = new Pose2d(new Vector2d(12, -64), Math.toRadians(90));

    ProfileAccelConstraint highAcc = new ProfileAccelConstraint(-30, 70);
    VelConstraint highSpeed = new TranslationalVelConstraint(70);

//    SequentialAction depositSample = new SequentialAction(
//            driveTrain.strafeTo(-57, -56),  // Deposit Sample
//            driveTrain.turnTo(225),
//            slides.moveTo(Slide.SlideState.HIGH_BUCKET.position),
//            driveTrain.strafeTo(-59, -59),
//            claw.setTo(Claw.ClawState.OPEN),
//            new SleepAction(1)
//    );

    public BlueLeftAuto(LinearOpMode opMode) {
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
                        wrist.moveTo(WristState.HOME.position - 50),
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position - 500),
                        driveTrain.strafeTo(4, -37, highAcc, highSpeed)
                ),
                driveTrain.strafeTo(4, -32, highAcc, highSpeed),
                slides.moveTo(5000),
                driveTrain.strafeTo(4, -40),

                // Push 2 Samples into Observation Station
                driveTrain.strafeTo(34, -33, 75),
                pusher.moveTo(PushMechanism.PushState.ACTIVE),
                new SleepAction(0.1),
                driveTrain.strafeTo(44, -60, 0),

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
                                driveTrain.strafeTo(60, -58),
                                driveTrain.strafeTo(63, -58)
                        )
                ),


                // Deposit Specimen on High Chamber
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position),
                        driveTrain.strafeTo(6, -36, 90)
                ),
                driveTrain.strafeTo(6, -33),
                slides.moveTo(5000),

                // Retrieve Specimen from Observation Station
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.CLIPPER.position),
                        new SequentialAction(
                                driveTrain.strafeTo(40, -64, 271)
                        )
                ),

                // Deposit Specimen on High Chamber
                new ParallelAction(
                        driveTrain.strafeTo(8, -36, 90),
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position)
                ),
                driveTrain.strafeTo(8, -33),
                slides.moveTo(5000),

                // Retrieve Specimen from Observation Station
                new ParallelAction(
                        slides.moveTo(Slide.SlideState.CLIPPER.position),
                        driveTrain.strafeTo(40, -64, 271)
                ),

                // Deposit Specimen on High Chamber
                new ParallelAction(
                        driveTrain.strafeTo(10, -36, 90),
                        slides.moveTo(Slide.SlideState.CLIP_HIGH_CHAMBER.position)
                ),
                driveTrain.strafeTo(10, -33),
                slides.moveTo(5000)

        ));
    }

    @Override
    public void stop() {
        FieldConstants.savePose(driveTrain.mecanumDrive.pose);
    }
}