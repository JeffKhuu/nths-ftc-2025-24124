package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Wrist;

@Config
@TeleOp(name = "Prototype", group = "ඞ")
public class Prototype extends LinearOpMode {
    public static double p = 0.022, i = 0, d = 0;
    public static double f = 0.02; // Feedforward constant
    public static int target = 0; // Target value in ticks

    public static double pos = 0.5;

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    Servo leftServo;
    Servo rightServo;
    Servo claw;
    Slide slide;
    Wrist wrist;

    PIDController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Init");
        telemetry.update();

        claw = hardwareMap.get(Servo.class, "claw");

        claw.setPosition(0.3);

        waitForStart();
        telemetry.addLine("Execute");
        //claw.setPosition(0.5);
        telemetry.update();
        if(isStopRequested() || !opModeIsActive()){
            telemetry.addLine("Stop");
            claw.setPosition(0.7);
            telemetry.update();
        }
    }

//    @Override
//    public void init() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////        slide = new Slide(hardwareMap);
////        wrist = new Wrist(hardwareMap);
////        claw = hardwareMap.get(Servo.class, "claw");
////        claw.setPosition(pos);
//    }
//
//    @Override
//    public void loop() {
//        CommandScheduler.getInstance().run();
//
//        //telemetry.addData("position", slide.leftSlide.getCurrentPosition());
//        //telemetry.addData("target", Slide.target);
//        telemetry.update();
//    }
}
