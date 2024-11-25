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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utilities.CarouselSelect;
import org.firstinspires.ftc.teamcode.utilities.Utilities;

/**
 * Example placeholder subsystem to represent viper slide/arm system
 */
@Config
public class Slide extends SubsystemBase {

    public final DcMotorEx leftSlide;
    public final DcMotorEx rightSlide;
    public final PIDController controller;
    public final VoltageSensor voltageSensor;

    public enum SlideState {
        HOME(0),
        ACTIVE(500),
        HOVER(1000),
        HANG(2650),
        HIGH_RUNG(3700),
        HIGH_BUCKET(10250);

        public final int position;

        SlideState(int position) {
            this.position = position;
        }
    }

    // 0.00075, 0.001, 0.00001
    public static double p = 0.00825, i = 0.00125, d = 0.00012;
    public static double f = 0; // Feedforward constant
    public static int target = 0; // Debugging Variable

    public final CarouselSelect<SlideState> positions = new CarouselSelect<>(SlideState.values());

    public Slide(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        // MAKE SURE THE DIRECTIONS ARE SET PROPERLY
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        positions.setSelected(0);
        controller = new PIDController(p, i, d);
        register();
    }

    @Override
    public void periodic() {
        controller.setPID(p, i, d);
        int armPos = leftSlide.getCurrentPosition(); // target
        double pid = controller.calculate(armPos, positions.getSelected().position);
        double power = (pid + f) * (12.0 / voltageSensor.getVoltage()); // Compensate for voltages

        setPower(power);
    }

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


    public void setPower(double power) {
        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }

}







