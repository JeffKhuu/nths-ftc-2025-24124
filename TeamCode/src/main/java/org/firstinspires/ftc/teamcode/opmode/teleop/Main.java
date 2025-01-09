package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.MotorWrist;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.utilities.ControllerEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryMaster;

@Config
@TeleOp(name = "Main TeleOp", group = "ඞ")
public class Main extends OpMode {
    @SuppressWarnings("unused")
    private ControllerEx driver, operator;

    private DriveTrain driveTrain;
    private Slide slides;
    private Claw claw;
    private MotorWrist wrist;

    private TelemetryEx telemetryEx;
    private TelemetryMaster telemetryMaster;

    // TODO: Reconstruct TelemetryEx w/ FTCDashboard

    @Override
    public void init() {
        //region Instantiate TeleOp Systems
        driveTrain = new RobotCentricDriveTrain(hardwareMap, FieldConstants.getLastSavedPose());
        slides = new Slide(hardwareMap);
        claw = new Claw(hardwareMap);
        wrist = new MotorWrist(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //endregion

        driver = ControllerEx.Builder(gamepad1)
                // Drive Train
                .bind(GamepadKeys.Button.LEFT_BUMPER, new InstantCommand(driveTrain.speeds::previous))
                .bind(GamepadKeys.Button.RIGHT_BUMPER, new InstantCommand(driveTrain.speeds::next))
                .bind(GamepadKeys.Button.START, new InstantCommand((driveTrain::resetHeading)))

                // Slides
                .bind(GamepadKeys.Button.DPAD_UP, new InstantCommand(slides.positions::next))
                .bind(GamepadKeys.Button.DPAD_DOWN, new InstantCommand(slides.positions::previous))
                .bind(GamepadKeys.Button.DPAD_RIGHT, new InstantCommand(() -> slides.positions.setSelected(8))) // Move the slides to the 7th position
                .bind(GamepadKeys.Button.DPAD_LEFT, new InstantCommand(() -> slides.positions.setSelected(1)))

                // Claw
                .bind(GamepadKeys.Button.X, claw.toggle())

                // Wrist
                .bind(GamepadKeys.Button.A, wrist.toggle())
                .bind(GamepadKeys.Button.BACK, new InstantCommand(() -> wrist.positions.setSelected(0)))
                .bind(GamepadKeys.Button.START, new InstantCommand(() -> {
                    wrist.startFlag = slides.startFlag = true;
                }))

                .build();

        operator = ControllerEx.Builder(gamepad2)
                // Slides
                .bind(GamepadKeys.Button.DPAD_UP, new InstantCommand(slides.positions::next))
                .bind(GamepadKeys.Button.DPAD_DOWN, new InstantCommand(slides.positions::previous))
                .bind(GamepadKeys.Button.DPAD_RIGHT, new InstantCommand(() -> slides.positions.setSelected(8))) // Move the slides to the 7th position
                .bind(GamepadKeys.Button.DPAD_LEFT, new InstantCommand(() -> slides.positions.setSelected(1)))

                // Claw
                .bind(GamepadKeys.Button.X, claw.toggle())

                // Wrist
                .bind(GamepadKeys.Button.A, wrist.toggle())
                .bind(GamepadKeys.Button.Y, new InstantCommand(() -> wrist.positions.setSelected(3)))
                .bind(GamepadKeys.Button.BACK, new InstantCommand(() -> wrist.positions.setSelected(0)))

                .build();


        //region Setup Extended Telemetry
        telemetryEx = new TelemetryEx(telemetry);
        telemetryMaster = new TelemetryMaster(telemetryEx); // fixme: May break things, idk
        telemetryMaster.subscribe(driveTrain)
                .subscribe(wrist)
                .subscribe(claw)
                .subscribe(slides);
        //endregion

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();



        double x = driver.getLeftX();
        double y = driver.getLeftY();
        double turn = driver.getRightX();
        driveTrain.move(x, y, turn);

        telemetryMaster.update(); //Updates telemetry for all subscribed systems
        telemetry.addData("\n\nRuntime", "%.2f", getRuntime());
    }

    @Override
    public void stop() {
        telemetryMaster.unsubscribe(driveTrain);
        telemetryMaster.unsubscribe(slides);
        telemetryMaster.unsubscribe(wrist);
        telemetryMaster.unsubscribe(claw);

        CommandScheduler.getInstance().unregisterSubsystem(driveTrain);
        CommandScheduler.getInstance().unregisterSubsystem(slides);
        CommandScheduler.getInstance().unregisterSubsystem(claw);
        CommandScheduler.getInstance().unregisterSubsystem(wrist);
    }
}
