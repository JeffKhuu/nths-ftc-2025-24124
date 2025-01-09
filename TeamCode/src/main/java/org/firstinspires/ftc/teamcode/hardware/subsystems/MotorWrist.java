package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utilities.Utilities;
import org.firstinspires.ftc.teamcode.utilities.selectors.ArraySelect;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

@Config
public class MotorWrist extends SubsystemBase implements TelemetrySubject {
    public static class Params {
        /* Operation mode for wrist.
            CONTROLLED allows the wrist to be controlled via code and gamepads
            MANUAL allows the wrist to only be controlled via RoadRunner parameters
        */
            public final WristMode MODE = WristMode.CONTROLLED;

        public int target =  0; // Debugging variable used for MANUAL mode

        private final String MOTOR_NAME = "wrist";

        public PIDFCoefficients pidf = new PIDFCoefficients(
                0.0022,
                0.005,
                0.0000,
                0.020
        );
    }

    public enum WristMode {
        CONTROLLED,
        MANUAL
    }

    public enum WristState{
        HOME(-100),
        INACTIVE(-250),
        ACTIVE(-550),
        HANG(-400);


        public final int position;
        WristState(int position){this.position = position;}

    }

    public static Params CONFIG = new Params();
    public final DcMotorEx wrist;
    public final PIDController controller;
    public final ArraySelect<WristState> positions = new ArraySelect<>(WristState.values());
    public boolean startFlag = false;

    public static double p = CONFIG.pidf.p, i = CONFIG.pidf.i, d = CONFIG.pidf.d, f = CONFIG.pidf.f;
    public int target = CONFIG.MODE == WristMode.MANUAL ? CONFIG.target : positions.getSelected().position;
    public static double ticks_in_degrees = 600 / 180.0;

    public MotorWrist(HardwareMap hardwareMap){
        controller = new PIDController(p, i, d);
        wrist = hardwareMap.get(DcMotorEx.class, CONFIG.MOTOR_NAME);

        wrist.setDirection(DcMotorSimple.Direction.FORWARD);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //positions.setSelected(0);
        register();
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {
        telemetry.print("Position", wrist.getCurrentPosition());
        telemetry.print("Target", target);
    }

    @Override
    public void periodic() {
        if(!startFlag) return;

        target = CONFIG.MODE == WristMode.MANUAL ? CONFIG.target : positions.getSelected().position;

        int armPos = wrist.getCurrentPosition();

        controller.setPID(p, i, d);
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;// Compensate for voltage discrepencies

        wrist.setPower(power);
    }

    /**
     * Toggle the current position between ACTIVE and INACTIVE
     * @return A FTCLib COmmand.
     */
        public Command toggle() {
        return new ToggleWrist(this);
    }

    /**
     * Move the slides to a given target position as a RaodRunner action.
     * @param target Target position in encoder ticks for the slides to travel to
     * @return A RoadRunner Action
     */
    public Action moveTo(int target) {
        return (TelemetryPacket packet) -> {
            int slidePos = wrist.getCurrentPosition();
            double tolerance = 0.01 * target + 1; // Check if we are within 1% of the target, with a constant of 1

            packet.put("Position", slidePos);
            packet.put("Target", target);
            packet.put("Position Reached?", Utilities.isBetween(slidePos, target + tolerance, target - tolerance));

            controller.setPID(p, i, d);
            double power = controller.calculate(slidePos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            wrist.setPower(power + ff);

            packet.put("Power", power);

            if (Utilities.isBetween(slidePos, target + tolerance, target - tolerance)) {
                wrist.setPower(power + ff);
                return false;
            } else {
                return true;
            }
        };
    }

    public Action setSelected(WristState state) {
        return (TelemetryPacket packet) -> {
            for(int i = 0; i < WristState.values().length; i++){
                if(WristState.values()[i] == state){
                    positions.setSelected(i);
                    return false;
                }
            }
            return true;
        };
    }


    public static class ToggleWrist extends CommandBase {
        private final MotorWrist wrist;

        public ToggleWrist(MotorWrist subsystem) {
            this.wrist = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            if(wrist.positions.getSelected() != WristState.ACTIVE){
                wrist.positions.setSelected(2);
            }else{
                wrist.positions.setSelected(1);
            }
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public void stopAndResetEncoders(){
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
