package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.CommandAction;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

/**
 * One servo based claw system
 * @version 1.0.0
 */
public class Claw extends SubsystemBase implements TelemetrySubject {
    public static final class Config {
        // Device name to retrieve from hardwareMap
        public static final String CLAW_NAME = "claw";

        // Default starting position of the claw
        public static ClawState start = ClawState.CLOSED;
    }

    public enum ClawState { // Accepts values between 0.5 and 1.0
        OPEN(0.60),
        CLOSED(1.0);

        public final double position;

        ClawState(double position) {
            this.position = position;
        }
    }

    public Servo claw;
    public boolean isClawOpen;

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, Config.CLAW_NAME);
        claw.setPosition(Config.start.position);
        isClawOpen = false;
    }


    @Override
    public void updateTelemetry(TelemetryEx telemetry) {
        telemetry.print("Position", claw.getPosition());
    }

    /**
     * Move the claw to a certain position given a ClawState to move to.
     * @param state A valid ClawState
     * @return A RoadRunner Action.
     */
    public Action moveTo(ClawState state) {
        return new CommandAction(setTo(state));
    }

    /**
     * Set the position of a claw to a given ClawState.
     * @param state A valid ClawState
     * @return A FTCLib Command
     */
    public Command setTo(ClawState state) {
        return new MoveClaw(state, this);
    }

    /**
     * Set the position of a claw to a given position between 0.0-1.0
     * @param position A double between 0.0 -> 1.0
     * @return A FTCLib Command
     */
    public Command setTo(Double position) {
        return new MoveClaw(position, this);
    }

    /**
     * Toggle the position of the claw between open and closed positions
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
