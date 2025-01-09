package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Wrist;

@Config
@TeleOp(name = "Prototype", group = "ඞ")
@SuppressWarnings("unused")
public class Prototype extends OpMode {
    public static double p = 0.022, i = 0, d = 0;
    public static double f = 0.02; // Feedforward constant
    public static int target = 0; // Target value in ticks

    public static double ticks_in_degrees = 600 / 180.0;

    DcMotorEx wrist;

    PIDController controller;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        wrist = hardwareMap.get(DcMotorEx.class, "wrist");
        wrist.setDirection(DcMotorSimple.Direction.FORWARD);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = wrist.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power = pid + ff;

        wrist.setPower(power);
        telemetry.addData("pos", armPos);
        telemetry.addData("target" , target);
    }
}
