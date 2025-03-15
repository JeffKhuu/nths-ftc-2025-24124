package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewMotorWrist;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;

@Config
@TeleOp(name = "StrafeDebugger", group = "ඞ")
public class StrafeDebugger extends LinearOpMode {
    DriveTrain driveTrain = null;
    NewMotorWrist wrist = null;
    Claw intake = null;
    Slide slide = null;

    public static double startX = 0;
    public static double startY = 0;
    public static double startHeading = 90;

    public static double x = startX;
    public static double y = startY;
    public static double heading = startHeading;
    public static boolean clawOpen = false;
    public static boolean armActive = false;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new RobotCentricDriveTrain(hardwareMap, new Pose2d(startX, startY, Math.toRadians(startHeading)));
        wrist = new NewMotorWrist(hardwareMap);
        intake = new Claw(hardwareMap);
        slide = new Slide(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            Actions.runBlocking(driveTrain.strafeTo(x, y));

        }
    }
}
