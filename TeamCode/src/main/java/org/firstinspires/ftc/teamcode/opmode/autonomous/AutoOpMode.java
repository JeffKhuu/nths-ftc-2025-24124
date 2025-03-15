package org.firstinspires.ftc.teamcode.opmode.autonomous;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.FieldConstants.ALLIANCE;
import org.firstinspires.ftc.teamcode.constants.FieldConstants.START_POSITION;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@Autonomous(name = "Auto OpMode", group = "ඞ", preselectTeleOp = "Main TeleOp")
public class AutoOpMode extends LinearOpMode {
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
            telemetry.update();
        }

        telemetry.clear();
        if (isStopRequested()) return;

        if (alliance == null || position == null) // If alliance and position are not specified, crash the opMode.
            throw new NullPointerException("Autonomous could not proceed. Position and/or Alliance values were not set.");

        AutoInstructions auto = new AutonomousFactory(this).buildAuto(alliance, position);

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Position", position);
        auto.autoInfoTelemetry(telemetry);
        telemetry.update();

        auto.init();

        CommandScheduler.getInstance().reset();

        CommandScheduler.getInstance().registerSubsystem(auto.driveTrain);
        CommandScheduler.getInstance().registerSubsystem(auto.slides);
        CommandScheduler.getInstance().registerSubsystem(auto.wrist);
        CommandScheduler.getInstance().registerSubsystem(auto.intake);
        CommandScheduler.getInstance().registerSubsystem(auto.pusher);

        waitForStart();
        auto.execute(); // Executes after the START (▶) button is pressed
        auto.stop();

        // Unregister all created subsystems
        CommandScheduler.getInstance().unregisterSubsystem(auto.driveTrain);
        CommandScheduler.getInstance().unregisterSubsystem(auto.slides);
        CommandScheduler.getInstance().unregisterSubsystem(auto.wrist);
        CommandScheduler.getInstance().unregisterSubsystem(auto.intake);
        CommandScheduler.getInstance().unregisterSubsystem(auto.pusher);
    }


    private void writeInitTelemetry() {
        telemetry.addLine("<!> PLEASE PROVIDE ALLIANCE AND STARTING POSITIONS IN ORDER FOR AUTONOMOUS TO WORK <!>");
        telemetry.addLine(String.format("%-22s %-22s", "Ⓧ for BLUE alliance", "Ⓑ for RED alliance"));
        telemetry.addLine(String.format("%-22s %-22s", "[←] for LEFT", "[→] for RIGHT "));
        telemetry.addLine("⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Position", position);
        telemetry.addLine("⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯");
        telemetry.addLine();
        telemetry.addLine("Press [START] to confirm.");
    }
}
