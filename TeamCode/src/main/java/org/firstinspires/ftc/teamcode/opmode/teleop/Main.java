package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.TestDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.utilities.CarouselSelect;
import org.firstinspires.ftc.teamcode.utilities.ControllerEx;
import org.firstinspires.ftc.teamcode.utilities.ToggleSelect;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryMaster;

import java.io.Console;

@TeleOp(name = "Main TeleOp", group = "ඞ")
public class Main extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private ControllerEx driver;

    private DriveTrain driveTrain;
    private Slide slides;
    private Claw claw;

    private TelemetryEx telemetryEx;
    private TelemetryMaster telemetryMaster;

    private final ToggleSelect<Claw.ClawState> clawStateToggle = new ToggleSelect<>(Claw.ClawState.OPEN, Claw.ClawState.CLOSED);
    private final CarouselSelect<Slide.SlideState> slideStateCarousel = new CarouselSelect<>(
            new Slide.SlideState[]{Slide.SlideState.HOME, Slide.SlideState.HIGH_BUCKET}
    );

    @Override
    public void init() {
        //region Instantiate TeleOp Systems
        driveTrain = new RobotCentricDriveTrain(hardwareMap, FieldConstants.getLastSavedPose()); // TODO: Factory Pattern?
        slides = new Slide(hardwareMap);
        claw = new Claw(hardwareMap);
        //endregion

        //region Register Gamepad Inputs
        driver = ControllerEx.Builder(gamepad1)
                .bind(GamepadKeys.Button.LEFT_BUMPER, new InstantCommand(() -> driveTrain.speeds.moveSelection(-1)))
                .bind(GamepadKeys.Button.RIGHT_BUMPER, new InstantCommand(driveTrain.speeds::moveSelection))
                //.bind(GamepadKeys.Button.B, slides.moveToPosition(Slide.SlideState.HIGH_BUCKET))

                .bindWhenHeld(GamepadKeys.Button.DPAD_UP, slides.extend()) // TODO: Test if THIS works- if not uncomment code and remove end() overrided method from commands
                //.bindWhenReleased(GamepadKeys.Button.DPAD_UP, new InstantCommand(slides::setZeroPower))

                .bindWhenHeld(GamepadKeys.Button.DPAD_DOWN, slides.retract())
                //.bindWhenReleased(GamepadKeys.Button.DPAD_UP, new InstantCommand(slides::setZeroPower))


                .bind(GamepadKeys.Button.X, claw.toggleClaw())

                //.bind(GamepadKeys.Button.START, new InstantCommand((driveTrain::resetHeading)))
                .build();
        //endregion

        //region Setup Extended Telemetry
        telemetryEx = new TelemetryEx(telemetry);
        telemetryMaster = new TelemetryMaster(telemetryEx)
                .subscribe(driveTrain);
        //endregion

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        telemetryMaster.update(); //Updates telemetry for all subscribed systems

        double x = driver.getLeftX();
        double y = driver.getLeftY();
        double turn = driver.getRightX();
        driveTrain.move(x, y, turn);

        if(gamepad1.b){
            clawStateToggle.toggle();
        }

        telemetryEx.print("Position", slides.leftSlide.getCurrentPosition());
        telemetryEx.print("Status", "Runtime: " + getRuntime());
    }


    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }
}
