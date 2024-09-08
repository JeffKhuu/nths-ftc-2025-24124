package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.utilities.CarouselSelect;
import org.firstinspires.ftc.teamcode.utilities.ControllerEx;

@TeleOp(name = "Main TeleOp", group = "ඞ")
public class Main extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    // TODO: Optimal way to tune Road Runner?
    // TODO: Take control hub, battery, etc home?

    ControllerEx driver; //Assigns gamepad1 as the driver gamepad
    //GamepadEx controller = new GamepadEx(gamepad2); //Assigns gamepad2 as the hardware gamepad

    DriveTrain driveTrain;

    private final CarouselSelect<Double> speedSelect = new CarouselSelect<>(
            new Double[]{1.0, 0.5, 0.25} // Speed multipliers
    );

    @Override
    public void init() {
        // Register gamepad inputs
        driver = new ControllerEx(gamepad1);
        driveTrain = new DriveTrain(hardwareMap);

        driver.getGamepadButton(GamepadKeys.Button.A) //Gamepad input test
                .whenPressed(new InstantCommand(() -> telemetry.addData("Command Based", "A button is pressed")))
                .whenHeld(new InstantCommand(() -> telemetry.addData("Command Based", "A button is being held")));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> speedSelect.moveSelection(-1)));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(speedSelect::moveSelection));


        driver.registerDPad(
                /*Up*/ new InstantCommand(() -> driveTrain.setDrivePower(DriveTrain.FORWARD)),
                /*Down*/ new InstantCommand(() -> driveTrain.setDrivePower(DriveTrain.BACKWARD))
        );

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {
        double x = driver.getLeftX();
        double y = driver.getLeftY(); // Values: -1 (Pull up) to 1 (Pull down)
        double turn = driver.getRightX();
        driveTrain.move(x, y, turn, speedSelect.getSelected());

        if(gamepad1.dpad_left){
            driveTrain.setDrivePower(DriveTrain.LEFT);
        }
        if(gamepad1.dpad_right){
            driveTrain.setDrivePower(DriveTrain.RIGHT);
        }


        CommandScheduler.getInstance().run();

        telemetry.addData("Speed", speedSelect.getSelected());
        telemetry.addData("Status", "Runtime: " + getRuntime());
    }


    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }
}
