package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Wrist;

@Config
@TeleOp(name = "Prototype TeleOp", group = "ඞ")
public class Prototype extends OpMode {
    public static double p = 0.022, i = 0, d = 0;
    public static double f = 0.02; // Feedforward constant
    public static int target = 0; // Target value in ticks

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    Servo leftServo;
    Servo rightServo;

    PIDController controller;

    @Override
    public void init() {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD); // MAKE SURE THE DIRECTIONS ARE SET PROPERLY
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftServo = hardwareMap.get(Servo.class, "leftWrist");
        rightServo = hardwareMap.get(Servo.class, "rightWrist");
        leftServo.setDirection(Servo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.FORWARD);
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = leftSlide.getCurrentPosition();
        double pid = controller.calculate(armPos, target);

        leftSlide.setPower(pid + f);
        rightSlide.setPower(pid + f);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
