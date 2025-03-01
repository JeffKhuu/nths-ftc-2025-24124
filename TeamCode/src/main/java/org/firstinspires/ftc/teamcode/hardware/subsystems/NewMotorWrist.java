package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

@Config
public class NewMotorWrist extends SubsystemBase implements TelemetrySubject {
    public static class Params {
        /* Operation mode for wrist.
            CONTROLLED allows the wrist to be controlled via code and gamepads
            MANUAL allows the wrist to only be controlled via RoadRunner parameters
        */
        public final WristMode MODE = WristMode.CONTROLLED;

        public int target = 0; // Debugging variable used for MANUAL mode

        private final String MOTOR_NAME = "wrist";
    }

    public enum WristMode {
        CONTROLLED,
        MANUAL
    }

    public enum WristState {
        HOME(0),
        INACTIVE(-150),
        ACTIVE(-480),
        HANG(-400);


        public final int position;

        WristState(int position) {
            this.position = position;
        }

    }

    public static Params CONFIG = new Params();
    public final DcMotorEx wrist;
    public boolean isActive; // True if the wrist is an "active" position

    public NewMotorWrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(DcMotorEx.class, CONFIG.MOTOR_NAME);

        wrist.setDirection(DcMotorSimple.Direction.FORWARD);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //positions.setSelected(0);
        register();
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {
        telemetry.print("Position", wrist.getCurrentPosition());
    }


    /**
     * Toggle the current position between ACTIVE and INACTIVE
     *
     * @return A FTCLib Command.
     */
    public Command toggle() {
        return new ToggleWrist(this);
    }

    /**
     * Set the position of the wrist to a given target position in ticks
     *
     * @return A FTCLib Command.
     */
    public Command setPositionTo(int target) {
        return new setPosition(this, target);
    }

    /**
     * Set the position of the wrist to a given WristState target position
     *
     * @return A FTCLib Command.
     */
    public Command setPositionTo(WristState target) {
        return new setPosition(this, target.position);
    }

    /**
     * Move the slides to a given target position as a RaodRunner action.
     *
     * @param target Target position in encoder ticks for the slides to travel to
     * @return A RoadRunner Action
     */
    public Action moveTo(int target) {
        return (TelemetryPacket packet) -> {
            wrist.setTargetPosition(target);
            wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPower(0.5);

            return false;
        };
    }

    public static class setPosition extends CommandBase {
        private final NewMotorWrist wrist;
        private final int position;

        public setPosition(NewMotorWrist subsystem, int position) {
            this.wrist = subsystem;
            this.position = position;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            wrist.wrist.setTargetPosition(position);
            wrist.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.wrist.setPower(0.5);
            wrist.isActive = true;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }


    public static class ToggleWrist extends CommandBase {
        private final NewMotorWrist wrist;

        public ToggleWrist(NewMotorWrist subsystem) {
            this.wrist = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            wrist.wrist.setTargetPosition(wrist.isActive ? WristState.INACTIVE.position : WristState.ACTIVE.position);
            wrist.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.wrist.setPower(0.5);
            wrist.isActive = !wrist.isActive;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public void stopAndResetEncoders() {
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
