package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.CommandAction;
import org.firstinspires.ftc.teamcode.utilities.selectors.ArraySelect;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

/**
 * One servo based claw system
 *
 * @version 1.0.1
 */
public class Claw extends SubsystemBase implements TelemetrySubject {
    // Device name to retrieve from hardwareMap
    private static final String CLAW_NAME = "claw";
    private static final String WRIST_NAME = "wrist2";

    // Default starting position of the claw
    private static final ClawState START = ClawState.CLOSED;

    public enum ClawState { // Accepts values between 0.5 and 1.0
        CLOSED(0),
        OPEN(0.5);

        public final double position;

        ClawState(double position) {
            this.position = position;
        }
    }

    public enum WristState {
        THIRTY(0),
        SIXTY(0.1),
        NINETY(0.2),
        ONETWENTY(0.3),
        ONEFIFTY(0.4),
        ONEEIGHTY(0.5),
        TWOTEN(0.6),
        TWOFORTY(0.7),
        TWOSEVENTY(0.8),
        THREEHUNDRED(1);

        public final double position;

        WristState(double position) {
            this.position = position;
        }
    }


    public Servo claw;
    public Servo wrist;
    public boolean isClawOpen;

    ArraySelect<WristState> pivots = new ArraySelect<>(WristState.values());

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, CLAW_NAME);
        wrist = hardwareMap.get(Servo.class, WRIST_NAME);

        claw.setPosition(START.position);
        wrist.setPosition(WristState.ONEEIGHTY.position);
        pivots.setSelected(5);
        isClawOpen = false;
    }


    @Override
    public void updateTelemetry(TelemetryEx telemetry) {
        telemetry.print("Position", claw.getPosition());
    }

    /**
     * Move the claw to a certain position given a ClawState to move to.
     *
     * @param state A valid ClawState
     * @return A RoadRunner Action.
     */
    public Action moveTo(ClawState state) {
        return (TelemetryPacket packet) -> {
            claw.setPosition(state.position);
            return false;
        };
    }

    public Action moveWrist(WristState state) {
        return (TelemetryPacket packet) -> {
            wrist.setPosition(state.position);
            return false;
        };
    }

    public Command setWristTo(WristState state) {
        return new MoveWrist(state, this);
    }

    public Command moveWrist(int amount) {
        return new PivotWrist(this, amount);
    }

    /**
     * Set the position of a claw to a given ClawState.
     *
     * @param state A valid ClawState
     * @return A FTCLib Command
     */
    public Command setTo(ClawState state) {
        return new MoveClaw(state, this);
    }

    /**
     * Set the position of a claw to a given position between 0.0-1.0
     *
     * @param position A double between 0.0 -> 1.0
     * @return A FTCLib Command
     */
    public Command setTo(Double position) {
        return new MoveClaw(position, this);
    }

    /**
     * Toggle the position of the claw between open and closed positions
     *
     * @return A FTCLib Command
     */
    public Command toggle() {
        return new ToggleClaw(this);
    }

    /**
     * FTCLib command that moves the claw to a specified claw state or position.
     * Uses the {@link Claw} Subsystem
     */
    public static class ToggleClaw extends CommandBase {
        Claw claw;

        public ToggleClaw(Claw subsystem) {
            claw = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void execute() {
            claw.claw.setPosition(!claw.isClawOpen ? ClawState.OPEN.position : ClawState.CLOSED.position);
            claw.isClawOpen = !claw.isClawOpen; // Toggle the position of the claw
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    /**
     * FTCLib command that moves the wrist to  wrist state or position.
     * Uses the {@link Claw} Subsystem
     */
    public static class PivotWrist extends CommandBase {
        Claw claw;
        int amount;

        public PivotWrist(Claw subsystem) {
            claw = subsystem;
            amount = 1;
            addRequirements(subsystem);
        }

        public PivotWrist(Claw subsystem, int amount) {
            claw = subsystem;
            this.amount = amount;
            addRequirements(subsystem);
        }


        @Override
        public void execute() {
            claw.pivots.moveSelection(amount);
            claw.wrist.setPosition(claw.pivots.getSelected().position);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    /**
     * FTCLib command that moves the wrist to  wrist state or position.
     * Uses the {@link Claw} Subsystem
     */
    public static class MoveWrist extends CommandBase {
        Claw claw;
        WristState state;

        public MoveWrist(WristState state, Claw subsystem) {
            claw = subsystem;
            this.state = state;
            addRequirements(subsystem);
        }

        @Override
        public void execute() {
            claw.pivots.next();
            claw.wrist.setPosition(state.position);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }


    /**
     * FTCLib command that moves the claw to a specified claw state or position.
     * Uses the {@link Claw} Subsystem
     */
    public static class MoveClaw extends CommandBase {
        private final Claw claw;
        private final double position;

        public MoveClaw(ClawState state, Claw subsystem) {
            claw = subsystem;
            position = state.position;
            addRequirements(claw);
        }

        public MoveClaw(double position, Claw subsystem) {
            claw = subsystem;
            this.position = position;
            addRequirements(claw);
        }

        @Override
        public void initialize() {
            claw.claw.setPosition(position);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

}
