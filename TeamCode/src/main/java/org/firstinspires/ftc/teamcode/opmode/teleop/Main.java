package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrainFieldCentric;
import org.firstinspires.ftc.teamcode.utilities.ControllerEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryMaster;

@TeleOp(name = "Main TeleOp", group = "ඞ")
public class Main extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    // TODO: Optimal way to tune Road Runner?
    // TODO: Take control hub, battery, etc home?

    private ControllerEx driver;

    private DriveTrainFieldCentric driveTrain;
    private TelemetryEx telemetryEx;
    private TelemetryMaster telemetryMaster;

    @Override
    public void init() {
        // Instantiate teleOp Systems
        driveTrain = new DriveTrainFieldCentric(hardwareMap);

        // Register gamepad inputs
        driver = ControllerEx.Builder(gamepad1)
                .bind(GamepadKeys.Button.A, new InstantCommand(() -> telemetry.addData("Command Based", "A button is pressed")))

                .bind(GamepadKeys.Button.LEFT_BUMPER, new InstantCommand(() -> driveTrain.speeds.moveSelection(-1)))
                .bind(GamepadKeys.Button.RIGHT_BUMPER, new InstantCommand(driveTrain.speeds::moveSelection))

                .bind(GamepadKeys.Button.START, new InstantCommand((driveTrain::resetHeading)))
                .build();

        // Setup extended telemetry
        telemetryEx = new TelemetryEx(telemetry);
        telemetryMaster = new TelemetryMaster(telemetryEx)
                .subscribe(driveTrain);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        telemetryMaster.update(); //Updates telemetry for all subscribed systems

        double x = driver.getLeftX();
        double y = driver.getLeftY(); // Values: -1 (Pull up) to 1 (Pull down)
        double turn = driver.getRightX();
        driveTrain.move(x, y, turn);

        telemetryEx.print("Status", "Runtime: " + getRuntime());
    }


    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }
}
