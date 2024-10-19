package org.firstinspires.ftc.teamcode.opmode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.FieldConstants.ALLIANCE;
import org.firstinspires.ftc.teamcode.constants.FieldConstants.START_POSITION;
import org.firstinspires.ftc.teamcode.opmode.autonomous.instructions.AutoInstructions;
import org.firstinspires.ftc.teamcode.opmode.autonomous.instructions.AutonomousFactory;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@Autonomous(name = "0+0 Auto", preselectTeleOp = "Main Teleop")
@AutonomousEx(preload = 0, cycles = 0, park = true)
public class AutoOpMode extends LinearOpMode {
    AutonomousEx autoData = getClass().getAnnotation(AutonomousEx.class);

    // Location
    boolean startPoseChosen = false;
    ALLIANCE alliance;
    START_POSITION position;

    @Override
    public void runOpMode() throws InterruptedException {
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
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Position", position);

        if (alliance == null || position == null) // If alliance and position are not specified, crash the opMode.
            throw new NullPointerException("AUTONOMOUS INITIALIZATION COULD NOT PROCEED, ALLIANCES AND STARTING POSITION WERE NOT SET.");

        AutoInstructions auto = new AutonomousFactory(this).buildAuto(alliance, position);
        auto.execute();
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
