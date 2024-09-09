package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrainFieldCentric;
import org.firstinspires.ftc.teamcode.utilities.ControllerEx;

@TeleOp(name = "Main TeleOp", group = "ඞ")
public class Main extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    // TODO: Optimal way to tune Road Runner?
    // TODO: Take control hub, battery, etc home?

    ControllerEx driver; //Assigns gamepad1 as the driver gamepad
    //GamepadEx controller = new GamepadEx(gamepad2); //Assigns gamepad2 as the hardware gamepad

    DriveTrainFieldCentric driveTrain;

    @Override
    public void init() {
        // Register gamepad inputs
        driveTrain = new DriveTrainFieldCentric(hardwareMap);

        //driver = new ControllerEx(gamepad1); // Create Gamepad and Register Button Inputs

        driver = ControllerEx.Builder(gamepad1)
                .bind(GamepadKeys.Button.A, new InstantCommand(() -> telemetry.addData("Command Based", "A button is pressed")))

                .bind(GamepadKeys.Button.LEFT_BUMPER, new InstantCommand(() -> driveTrain.speeds.moveSelection(-1)))
                .bind(GamepadKeys.Button.RIGHT_BUMPER, new InstantCommand(driveTrain.speeds::moveSelection))

                .bind(GamepadKeys.Button.START, new InstantCommand((driveTrain::resetHeading)))
                .build();


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        double x = driver.getLeftX();
        double y = driver.getLeftY(); // Values: -1 (Pull up) to 1 (Pull down)
        double turn = driver.getRightX();
        driveTrain.move(x, y, turn);

        telemetry.addData("Speed", driveTrain.speeds.getSelected());
        telemetry.addData("Status", "Runtime: " + getRuntime());
        telemetry.addData("Heading", driveTrain.getBotHeading());
    }


    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }
}
