package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.MotorWrist;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewMotorWrist;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Wrist;

@TeleOp(name = "Reset Encoders")
public class ResetMotorEncoders extends LinearOpMode {
    Slide slides;
    NewMotorWrist wrist;

    @Override
    public void runOpMode() {
        slides = new Slide(hardwareMap);
        wrist = new NewMotorWrist(hardwareMap);

        CommandScheduler.getInstance().unregisterSubsystem(slides);
        CommandScheduler.getInstance().unregisterSubsystem(wrist);
        telemetry.addLine("Press START [▶] to reset the encoders on select encoders");
        telemetry.addLine();
        telemetry.addLine("To view all encoders that are reset see ResetMotorEncoders.java. To add motors to reset in ResetMotorEncoders.java");

        waitForStart();
        slides.stopAndResetEncoders(); // Reset Viper Slide Encoders
        wrist.stopAndResetEncoders();

        CommandScheduler.getInstance().unregisterSubsystem(slides);
        CommandScheduler.getInstance().unregisterSubsystem(wrist);
    }
}
