package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utilities.Utilities;
import org.firstinspires.ftc.teamcode.utilities.selectors.ArraySelect;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

import java.util.Locale;

/**
 * Two motor viper-slide based slide subsystem
 *
 * @version 1.0.1
 */
@Config
public class Slide extends SubsystemBase implements TelemetrySubject {
    /* Operation mode for slides.
       CONTROLLED allows the slides to be controlled via code and gamepads
       MANUAL allows the slides to only be controlled via RoadRunner parameters
     */
    private final static SlideMode MODE = SlideMode.CONTROLLED;

    //Device names to retrieve from hardwareMap
    private static final String LEFT_MOTOR_NAME = "leftSlide";
    private static final String RIGHT_MOTOR_NAME = "rightSlide";

    private final DcMotorEx leftSlide, rightSlide;
    private final PIDController controller;
    private final VoltageSensor voltageSensor;
    private boolean startFlag = false;

    public static int target = 0; // Debugging variable used in MANUAL mode
    public static PIDFCoefficients coefficients = new PIDFCoefficients(
            0.00945,
            0,
            0.0001,
            0.01
    );

    public enum SlideState {
        HOME(0),
        ACTIVE(750), // 380
        //INBETWEEN(800),
        //HOVER(1200), // 760
        CLIPPER(1600), //1650
        //HANG(2650), // 1700
        HIGH_RUNG(3700), // 2000
        //CLIP_HANG(5000), //3200
        CLIP_HIGH_CHAMBER(7000), // 4000
        HIGH_BUCKET(10000); //5800

        public final int position;
        public static final int MAX = SlideState.values().length - 1;

        SlideState(int position) {
            this.position = position;
        }
    }

    public enum SlideMode {
        MANUAL,
        CONTROLLED
    }

    // Unpack PIDF coefficients from config
    public static double p = coefficients.p, i = coefficients.i, d = coefficients.d, f = coefficients.f;

    // Create a selection array from all SlideState values
    public final ArraySelect<SlideState> positions = new ArraySelect<>(SlideState.values());


    public Slide(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, LEFT_MOTOR_NAME); // Retrieve device names from Config
        rightSlide = hardwareMap.get(DcMotorEx.class, RIGHT_MOTOR_NAME);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        // MAKE SURE THE DIRECTIONS ARE SET PROPERLY
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        positions.setSelected(0);
        controller = new PIDController(p, i, d);
        register();
    }

    @Override
    public void periodic() {
        if (!startFlag) return;

        int target = (MODE != SlideMode.MANUAL) ? positions.getSelected().position : Slide.target;

        controller.setPID(p, i, d);
        int armPos = leftSlide.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double power = (pid + f) * (12.0 / voltageSensor.getVoltage()); // Compensate for voltage discrepencies

        leftSlide.setPower(power);
        rightSlide.setPower(power);

        if(rightSlide.getCurrentPosition() < 500 && positions.getSelected() == SlideState.HOME){
            rightSlide.setPower(0);
        }
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {
        int target = (MODE != SlideMode.MANUAL) ? positions.getSelected().position : Slide.target;

        telemetry.print("Target", target);
        telemetry.print(String.format(Locale.CANADA, "Left Slide Pos: %d | Right Slide Pos: %d",
                leftSlide.getCurrentPosition(), rightSlide.getCurrentPosition()));
        telemetry.print(coefficients.toString());

        if (positions.getSelected().position != 0) {
            telemetry.print("Error", Utilities.calculateErr(target, leftSlide.getCurrentPosition()));
        }
    }

    /**
     * Move the slides to a given target position as a RaodRunner action.
     *
     * @param target Target position in encoder ticks for the slides to travel to
     * @return A RoadRunner Action
     */
    public Action moveTo(int target) {
        return (TelemetryPacket packet) -> {
            int slidePos = leftSlide.getCurrentPosition();
            double tolerance = 0.01 * target + 10; // Check if we are within 1% of the target, with a constant of 1

            // FTCDashboard Telemetry
            packet.put("Position", slidePos);
            packet.put("Target", target);
            packet.put("Position Reached?", Utilities.isBetween(slidePos, target - tolerance, target + tolerance));

            controller.setPID(p, i, d);
            double pid = controller.calculate(slidePos, target);
            double power = (pid + f + 0.4) * (12.0 / voltageSensor.getVoltage()); // Compensate for voltages
            setPower(power);

            packet.put("Power", power);

            if (Utilities.isBetween(slidePos, target - tolerance, target + tolerance)) {
                setPower(f);
                return false; // Stop the command
            } else {
                return true; // Otherwise continue running it
            }
        };
    }


    /**
     * Set the power of both motors driving the slides given a power
     *
     * @param power Power to give to the motors
     */
    public void setPower(double power) {
        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }

    /**
     * Stop and reset all used motors. Sets motor's RunMode to RUN_WITHOUT_ENCODER after.
     */
    public void stopAndResetEncoders() {
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set the slides start flag to true to start incorporating periodic PIDF control
     */
    public void triggerStartFlag() {
        startFlag = true;
    }
}







