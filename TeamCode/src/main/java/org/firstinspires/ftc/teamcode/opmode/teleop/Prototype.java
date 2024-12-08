package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    public static double pos = 0.5;

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    Servo leftServo;
    Servo rightServo;
    Claw claw;
    Slide slide;
    Wrist wrist;

    PIDController controller;

    @Override
    public void init() {
        claw = new Claw(hardwareMap);
        slide = new Slide(hardwareMap);
        wrist = new Wrist(hardwareMap);
    }

    @Override
    public void loop() {
        //gamepad1.right_trigger
    }
}
