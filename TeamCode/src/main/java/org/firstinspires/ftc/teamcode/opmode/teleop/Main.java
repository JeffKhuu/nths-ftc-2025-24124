package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.utilities.ControllerEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryMaster;

@TeleOp(name = "Main TeleOp", group = "ඞ")
public class Main extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private ControllerEx driver;

    private DriveTrain driveTrain;
    private Slide slides;
    private Claw claw;

    private TelemetryEx telemetryEx;
    private TelemetryMaster telemetryMaster;

    // TODO: Test RIGHT_STICK_BUTTON turn 90 degrees
    // TODO: Test Better Drive Train
    // TODO: Test DPAD_LEFT & DPAD_RIGHT sliding
    // TODO: Test new claw toggle
    // TODO: Test speed changing
    // TODO: Test Telemetry

    @Override
    public void init() {
        //region Instantiate TeleOp Systems
        driveTrain = new RobotCentricDriveTrain(hardwareMap, FieldConstants.getLastSavedPose()); // TODO: Factory Pattern?
        slides = new Slide(hardwareMap);
        claw = new Claw(hardwareMap);
        //endregion

        driver = ControllerEx.Builder(gamepad1)
                // Speed Control
                .bind(GamepadKeys.Button.LEFT_BUMPER, new InstantCommand(driveTrain.speeds::next))
                .bind(GamepadKeys.Button.RIGHT_BUMPER, new InstantCommand(driveTrain.speeds::previous))

                // Slides
                .bindWhenHeld(GamepadKeys.Button.DPAD_UP, slides.extend())
                .bindWhenHeld(GamepadKeys.Button.DPAD_DOWN, slides.retract())
                .bind(GamepadKeys.Button.DPAD_LEFT, slides.moveTo(slides.positions.next().getSelected()))
                .bind(GamepadKeys.Button.DPAD_RIGHT, slides.moveTo(slides.positions.previous().getSelected()))

                .bind(GamepadKeys.Button.B, slides.moveTo(slides.positions.getSelected()))
                .bind(GamepadKeys.Button.RIGHT_STICK_BUTTON, driveTrain.turn(90))

                // Claw
                .bind(GamepadKeys.Button.X, claw.toggleClaw())

                //.bind(GamepadKeys.Button.START, new InstantCommand((driveTrain::resetHeading)))
                .build();

        //region Setup Extended Telemetry
        telemetryEx = new TelemetryEx(telemetry);
        telemetryMaster = new TelemetryMaster(telemetryEx)
                .subscribe(driveTrain)
                .subscribe(slides)
                .subscribe(claw);
        //endregion

        telemetryEx.print("Status", "Initialized");
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        telemetryMaster.update(); //Updates telemetry for all subscribed systems

        double x = driver.getLeftX();
        double y = driver.getLeftY();
        double turn = driver.getRightX();
        driveTrain.move(x, y, turn); // TODO: Snap to 90 degree turns

        telemetryEx.print("Status", "Runtime: " + getRuntime());
    }
}
