package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
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
    private DcMotorEx slideMotor;

    private TelemetryEx telemetryEx;
    private TelemetryMaster telemetryMaster;

    @Override
    public void init() {
        // Instantiate teleOp Systems
        driveTrain = new DriveTrain(hardwareMap, FieldConstants.getLastSavedPose());
        slides = new Slide(hardwareMap);
        claw = new Claw(hardwareMap);

        slideMotor = hardwareMap.get(DcMotorEx.class, "leftSlide");

        // Register gamepad inputs
        driver = ControllerEx.Builder(gamepad1)
                .bind(GamepadKeys.Button.LEFT_BUMPER, new InstantCommand(() -> driveTrain.speeds.moveSelection(-1)))
                .bind(GamepadKeys.Button.RIGHT_BUMPER, new InstantCommand(driveTrain.speeds::moveSelection))

                //.bindWhileHeld(GamepadKeys.Button.DPAD_UP, slides.extend())
                //.bindWhileHeld(GamepadKeys.Button.DPAD_DOWN, slides.retract())

                .bind(GamepadKeys.Button.A, claw.moveClaw(Claw.ClawState.OPEN))
                .bind(GamepadKeys.Button.B, claw.moveClaw(Claw.ClawState.CLOSED))

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

        if(gamepad1.dpad_up){
            slideMotor.setPower(0.75);
        }else if(gamepad1.dpad_down){
            slideMotor.setPower(-0.75);
        }
        else{
            slideMotor.setPower(0);
        }

        telemetryEx.print("Status", "Runtime: " + getRuntime());
    }


    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }
}
