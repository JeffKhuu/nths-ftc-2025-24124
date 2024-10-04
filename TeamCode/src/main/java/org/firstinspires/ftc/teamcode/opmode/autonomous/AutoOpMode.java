package org.firstinspires.ftc.teamcode.opmode.autonomous;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.constants.FieldConstants.ALLIANCE;
import org.firstinspires.ftc.teamcode.constants.FieldConstants.START_POSITION;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@Autonomous(name = "0+0 Auto", preselectTeleOp = "Main Teleop")
@AutonomousEx(preload = 0, cycles = 0, park = true)
public class AutoOpMode extends LinearOpMode {
    AutonomousEx autoData = getClass().getAnnotation(AutonomousEx.class);

    // Location
    boolean startPoseChosen = false;
    Pose2d startPose;
    ALLIANCE alliance;
    START_POSITION position;

    // Subsystems
    DriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        //region Initialize starting position
        while (opModeInInit() && !startPoseChosen) {
            // Initiate AUTO alliance side and starting position
            if (gamepad1.x) alliance = ALLIANCE.BLUE;
            if (gamepad1.b) alliance = ALLIANCE.RED;

            if (gamepad1.dpad_left) position = START_POSITION.ALLIANCE_LEFT;
            if (gamepad1.dpad_right) position = START_POSITION.ALLIANCE_RIGHT;

            if (gamepad1.start) startPoseChosen = true;
            writeInitTelemetry(); // Add init telemetry
        }

        if (isStopRequested()) return;
        telemetry.clear();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        if (alliance == null || position == null) // If alliance and position are not specified, crash the opMode.
            throw new NullPointerException("AUTONOMOUS INITIALIZATION COULD NOT PROCEED, ALLIANCES AND STARTING POSITION WERE NOT SET.");

        if (alliance == ALLIANCE.RED && position == START_POSITION.ALLIANCE_LEFT)
            startPose = new Pose2d(0, 0, 0); // RED ALLIANCE LEFT
        else if (alliance == ALLIANCE.RED && position == START_POSITION.ALLIANCE_RIGHT)
            startPose = new Pose2d(0, 0, 0); // RED ALLIANCE RIGHT
        else if (alliance == ALLIANCE.BLUE && position == START_POSITION.ALLIANCE_LEFT)
            startPose = new Pose2d(0, 0, 0); // BLUE ALLIANCE LEFT
        else if (alliance == ALLIANCE.BLUE && position == START_POSITION.ALLIANCE_RIGHT)
            startPose = new Pose2d(0, 0, 0); // BLUE ALLIANCE RIGHT
        //endregion

        // Instantiate subsystems
        driveTrain = new FieldCentricDriveTrain(hardwareMap, startPose);

        // Build test paths & actions
        Action testAction = driveTrain.mecanumDrive.actionBuilder(driveTrain.mecanumDrive.pose)
                .strafeTo(new Vector2d(0, 24))
                .build();

        waitForStart(); // Executes after the START (▶) button is pressed

        Actions.runBlocking( // Run a specified sequence of actions
                new SequentialAction(
                        testAction
                )
        );


        // Executes after the OpMode has completed all above tasks
        if (isStopRequested()) {
            FieldConstants.savePose(driveTrain.mecanumDrive.pose);
        }
    }


    private void writeInitTelemetry() {
        telemetry.addLine("<!> PLEASE PROVIDE ALLIANCE AND STARTING POSITIONS IN ORDER FOR AUTONOMOUS TO WORK <!>");
        telemetry.addLine("⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Position", position);
        telemetry.addLine("⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯");
        telemetry.addData("Preload", autoData.preload());
        telemetry.addData("Cycles", autoData.cycles());
        telemetry.addData("Park?", autoData.park());
        telemetry.addLine();
        telemetry.addLine("Press [START] to confirm.");
        telemetry.update();
    }
}
