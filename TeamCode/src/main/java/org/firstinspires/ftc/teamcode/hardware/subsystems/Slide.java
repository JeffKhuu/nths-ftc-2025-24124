package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utilities.CarouselSelect;

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
        ACTIVE(700),
        LOW_RUNG(3000),
        LOW_BUCKET(5000),
        HIGH_RUNG(7000),
        HIGH_BUCKET(10250);

        public final int position;

        SlideState(int position) {
            this.position = position;
        }
    }

    public static double p = 0.003, i = 0, d = 0.00001;
    public static double f = 0; // Feedforward constant
    //public static int target = 0;

    public final CarouselSelect<SlideState> positions = new CarouselSelect<>(
            new SlideState[]{
                    SlideState.HOME,
                    SlideState.ACTIVE,
                    SlideState.LOW_RUNG,
                    SlideState.LOW_BUCKET,
                    SlideState.HIGH_RUNG,
                    SlideState.HIGH_BUCKET
            }
    ); //FIXME

    public Slide(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        // MAKE SURE THE DIRECTIONS ARE SET PROPERLY
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        positions.setSelected(1);
        controller = new PIDController(p, i, d);
    }

    @Override
    public void periodic() { // TODO: Un-limited slide opMode
        controller.setPID(p, i, d);
        int armPos = leftSlide.getCurrentPosition();
        double pid = controller.calculate(armPos, positions.getSelected().position);
        double power = (pid + f) * (12.0 / voltageSensor.getVoltage()); // Compensate for voltages

        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }


    public void setZeroPower() {
        rightSlide.setPower(0);
        leftSlide.setPower(0);
    }

}







