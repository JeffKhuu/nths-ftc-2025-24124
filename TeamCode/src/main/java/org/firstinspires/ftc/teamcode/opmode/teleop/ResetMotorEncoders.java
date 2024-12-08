package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;

@TeleOp(name = "Reset Encoders")
public class ResetMotorEncoders extends LinearOpMode {
    Slide slides;

    @Override
    public void runOpMode() {
        slides = new Slide(hardwareMap);
        telemetry.addLine("Press START [▶] to reset the encoders on select encoders");
        telemetry.addLine();
        telemetry.addLine("To view all encoders that are reset see ResetMotorEncoders.java. To add motors to reset in ResetMotorEncoders.java");

        waitForStart();
        slides.stopAndResetEncoders(); // Reset Viper Slide Encoders
    }
}
