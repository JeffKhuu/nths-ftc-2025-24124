package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewMotorWrist;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PushMechanism;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
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
    private NewMotorWrist wrist;
    private Claw claw;
    private PushMechanism pusher;

    private TelemetryEx telemetryEx;
    private TelemetryMaster telemetryMaster;

    @Override
    public void init() {
        //region Instantiate TeleOp Systems
        driveTrain = new RobotCentricDriveTrain(hardwareMap, FieldConstants.getLastSavedPose());
        slides = new Slide(hardwareMap);
        wrist = new NewMotorWrist(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        claw = new Claw(hardwareMap);
        pusher = new PushMechanism(hardwareMap);
        //endregion

        driver = ControllerEx.Builder(gamepad1)
                // Drive Train
                .bind(Button.LEFT_BUMPER, new InstantCommand(driveTrain.getSpeeds()::previous))
                .bind(Button.RIGHT_BUMPER, new InstantCommand(driveTrain.getSpeeds()::next))
                .bind(Button.START, new InstantCommand((driveTrain::resetHeading)))

                .bind(Button.X, pusher.setTo(PushMechanism.PushState.ACTIVE))
                .bind(Button.B, pusher.setTo(PushMechanism.PushState.INACTIVE))

                .build();

        operator = ControllerEx.Builder(gamepad2)
                // Slides
                .bind(Button.DPAD_UP, new InstantCommand(slides.positions::next))
                .bind(Button.DPAD_DOWN, new InstantCommand(slides.positions::previous))
                .bind(Button.DPAD_RIGHT, new InstantCommand(() -> slides.positions.setSelected(Slide.SlideState.MAX))) // Move the slides to the 7th position
                .bind(Button.DPAD_LEFT, new InstantCommand(() -> slides.positions.setSelected(1)))
                .bind(Button.Y, new InstantCommand(() -> {
                    slides.stopAndResetEncoders();
                }))

                // Intake
                .bind(Button.X, claw.toggle())
                .bind(Button.LEFT_BUMPER, claw.moveWrist(1))
                .bind(Button.RIGHT_BUMPER, claw.moveWrist(-1))

                // Wrist
                .bind(Button.A, wrist.toggle())
                .bind(Button.BACK, wrist.setPositionTo(NewMotorWrist.WristState.HOME))
                .bind(Button.START, new InstantCommand(() -> {
                    slides.triggerStartFlag();
                    CommandScheduler.getInstance().schedule(wrist.setPositionTo(NewMotorWrist.WristState.HOME));
                }))

                .build();

        //region Setup Extended Telemetry
        telemetryEx = new TelemetryEx(telemetry);
        telemetryMaster = new TelemetryMaster(telemetryEx);
        telemetryMaster
                .subscribe(driveTrain)
                .subscribe(wrist)
                .subscribe(claw)
                .subscribe(slides);
        //endregion

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        FieldConstants.savePose(driveTrain.mecanumDrive.pose);

        double x = driver.getLeftX();
        double y = driver.getLeftY();
        double turn = driver.getRightX();
        driveTrain.move(x, y, turn);

        telemetryMaster.update(); //Updates telemetry for all subscribed systems
        telemetry.addData("\n\nRuntime", "%.2f", getRuntime());
    }

    @Override
    public void stop() {
        // IMPORTANT! UNSUBSCRIBE AND UNREGISTER ALL SUBSYSTEMS
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
