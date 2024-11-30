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
public class Prototype extends OpMode {
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
    public void init() {
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0.8);

    }

    @Override
    public void loop() {
        if(gamepad1.x){
            claw.setPosition(0.6);
        }else{
            claw.setPosition(0.8);
        }

        //gamepad1.right_trigger
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
