package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.utilities.ControllerEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryMaster;

@Config
@TeleOp(name = "Main TeleOp", group = "ඞ")
public class Main extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private ControllerEx driver;

    private DriveTrain driveTrain;
    private Slide slides;
    private Claw claw;
    private Wrist wrist;

    //private TelemetryEx telemetryEx;
    //private TelemetryMaster telemetryMaster;

    // TODO: Reconstruct TelemetryEx w/ FTCDashboard
    // TODO: Move slides by set ticks

    @Override
    public void init() {
        //region Instantiate TeleOp Systems
        driveTrain = new RobotCentricDriveTrain(hardwareMap, FieldConstants.getLastSavedPose()); // TODO: Factory Pattern?
        slides = new Slide(hardwareMap);
        claw = new Claw(hardwareMap);
        wrist = new Wrist(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());;
        //endregion

        driver = ControllerEx.Builder(gamepad1)
                // Drive Train
                .bind(GamepadKeys.Button.LEFT_BUMPER, new InstantCommand(driveTrain.speeds::previous))
                .bind(GamepadKeys.Button.RIGHT_BUMPER, new InstantCommand(driveTrain.speeds::next))
                .bind(GamepadKeys.Button.START, new InstantCommand((driveTrain::resetHeading)))

                // Slides
                .bind(GamepadKeys.Button.DPAD_UP, new InstantCommand(slides.positions::next))
                .bind(GamepadKeys.Button.DPAD_DOWN, new InstantCommand(slides.positions::previous))

                // Claw
                .bind(GamepadKeys.Button.X, claw.toggle())

                // Wrist
                .bind(GamepadKeys.Button.A, wrist.toggle())
                .bind(GamepadKeys.Button.BACK, wrist.moveWrist(Wrist.WristState.HOME))

                .build();

        //region Setup Extended Telemetry
        //endregion

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        //telemetryMaster.update(); //Updates telemetry for all subscribed systems

        double x = driver.getLeftX();
        double y = driver.getLeftY();
        double turn = driver.getRightX();
        driveTrain.move(x, y, turn); // TODO: Snap to 90 degree turns

//        telemetryEx.print("⎯⎯⎯⎯⎯⎯⎯⎯SLIDES⎯⎯⎯⎯⎯⎯⎯⎯");
//        telemetryEx.printCarousel(slides.positions);
//        telemetryEx.print("Target", slides.positions.getSelected());
//        telemetryEx.print("Left Position", slides.leftSlide.getCurrentPosition());
//        telemetryEx.print("Right Position", slides.rightSlide.getCurrentPosition());

        telemetry.addData("Wrist Active?", wrist.isActive);

        telemetry.addData("Target", slides.positions.getSelected().position);
        telemetry.addData("Pos", slides.leftSlide.getCurrentPosition());
        if(slides.positions.getSelected().position != 0){
            telemetry.addData("Error", (((double)slides.positions.getSelected().position - (double)slides.leftSlide.getCurrentPosition())/(double)slides.positions.getSelected().position));
        }

        telemetry.addData("Runtime", "%.2f", getRuntime());
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().unregisterSubsystem(driveTrain);
        CommandScheduler.getInstance().unregisterSubsystem(slides);
        CommandScheduler.getInstance().unregisterSubsystem(claw);
        CommandScheduler.getInstance().unregisterSubsystem(wrist);
    }
}
