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
 * @version 1.0.0
 */
@Config
public class Slide extends SubsystemBase implements TelemetrySubject {
    public static final class Config {
        /* Operation mode for slides.
            CONTROLLED allows the slides to be controlled via code and gamepads
            MANUAL allows the slides to only be controlled via RoadRunner parameters
        */
        public static SlideMode MODE = SlideMode.CONTROLLED;
        public static int target = 0; // Debugging variable used in MANUAL mode

        //Device names to retrieve from hardwareMap
        public static final String LEFT_MOTOR_NAME = "leftSlide";
        public static final String RIGHT_MOTOR_NAME = "rightSlide";

        // PIDF Controller Coefficients
        // Old Coefficients: 0.00825, 0.00125, 0.00012, 0
        public static PIDFCoefficients coefficients = new PIDFCoefficients(
                0.005,
                0.1,
                0.00025,
                0.09
        );
    }


    public final DcMotorEx leftSlide, rightSlide;
    public final PIDController controller;
    public final VoltageSensor voltageSensor;

    public enum SlideState {
        HOME(0),
        ACTIVE(380), // 500
        HOVER(760), // 1000
        CLIPPER(1000), //1500
        HANG(1700), // 2650
        HIGH_RUNG(2000), // 3700
        CLIP_HANG(3200), //5000
        CLIP_HIGH_CHAMBER(4000), // 7000
        HIGH_BUCKET(5800); //10250

        public final int position;

        SlideState(int position) {
            this.position = position;
        }
    }
    public enum SlideMode {
        MANUAL,
        CONTROLLED
    }

    // Unpack PIDF coefficients from config
    public static double p = Config.coefficients.p, i = Config.coefficients.i, d = Config.coefficients.d, f = Config.coefficients.f;

    // Create a selection array from all SlideState values
    public final ArraySelect<SlideState> positions = new ArraySelect<>(SlideState.values());

    public Slide(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, Config.LEFT_MOTOR_NAME); // Retrieve device names from Config
        rightSlide = hardwareMap.get(DcMotorEx.class, Config.RIGHT_MOTOR_NAME);
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
        int target = (Config.MODE != SlideMode.MANUAL) ? positions.getSelected().position : Config.target;

        controller.setPID(p, i, d);
        int armPos = leftSlide.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double power = (pid + f) * (12.0 / voltageSensor.getVoltage()); // Compensate for voltage discrepencies

        setPower(power);
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {
        int target = (Config.MODE != SlideMode.MANUAL) ? positions.getSelected().position : Config.target;

        telemetry.print("Target", target);
        telemetry.print(String.format(Locale.CANADA, "Left Slide Pos: %d | Right Slide Pos: %d",
                leftSlide.getCurrentPosition(), rightSlide.getCurrentPosition()));
        telemetry.print(Config.coefficients.toString());

        if(positions.getSelected().position != 0){
            telemetry.print("Error", Utilities.calculateErr(target, leftSlide.getCurrentPosition()));
        }
    }

    /**
     * Move the slides to a given target position as a RaodRunner action.
     * @param target Target position in encoder ticks for the slides to travel to
     * @return A RoadRunner Action
     */
    public Action moveTo(int target) {
        return (TelemetryPacket packet) -> {
            int slidePos = leftSlide.getCurrentPosition();
            double tolerance = 0.01 * target + 1; // Check if we are within 1% of the target, with a constant of 1

            packet.put("Position", slidePos);
            packet.put("Target", target);
            packet.put("Position Reached?", Utilities.isBetween(slidePos, target - tolerance, target + tolerance));

            controller.setPID(p, i, d);
            double pid = controller.calculate(slidePos, target);
            double power = (pid + f) * (12.0 / voltageSensor.getVoltage()); // Compensate for voltages
            setPower(power);

            packet.put("Power", power);

            if (Utilities.isBetween(slidePos, target - tolerance, target + tolerance)) {
                setPower(f);
                return false;
            } else {
                return true;
            }
        };
    }


    /**
     * Set the power of both motors driving the slides given a power
     * @param power Power to give to the motors
     */
    public void setPower(double power) {
        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }

    public void stopAndResetEncoders(){
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}







