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

import java.util.Locale;

/**
 * Two servo based Wrist sysstem.
 * @version 1.0.0
 */
public class Wrist extends SubsystemBase implements TelemetrySubject {
    //region Configs and Constants
    public static final class Config {
        // Device Name to retrieve from hardwareMap
        public static final String LEFT_SERVO = "leftWrist";
        public static final String RIGHT_SERVO = "rightWrist";

        // Starting servo position
        public static WristState start = WristState.HOME;
    }

    public enum WristState {
        HOME(0.72),
        INACTIVE(0.52),
        HANG(0.45),
        ACTIVE(0.16);

        final double position;

        WristState(double position) {
            this.position = position;
        }
    }
    //endregion

    public final Servo leftServo, rightServo;
    public boolean isActive; // True if the wrist is an "active" position

    public Wrist(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, Config.LEFT_SERVO);
        rightServo = hardwareMap.get(Servo.class, Config.RIGHT_SERVO);

        leftServo.setDirection(Servo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.FORWARD);

        leftServo.setPosition(Config.start.position);
        rightServo.setPosition(Config.start.position);
        isActive = true;
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {
        telemetry.print("Wrist Position", leftServo.getPosition());
        telemetry.print("Wrist Engaged: ", isActive);
    }

    /**
     * Move the wrist to a certain position given a WristState to move to.
     * @param state A valid WristState
     * @return A RoadRunner Action.
     */
    public Action moveTo(WristState state){ // Imperative
        return new CommandAction(setTo(state));
    }

    /**
     * Set the position of the wrist to a given position given a WristState to move to.
     * @param state A valid WristState
     * @return A FTCLib Command.
     */
    public Command setTo(WristState state) { // Declarative
        return new SetWristPosition(state, this);
    }

    /**
     * Toggle the current position between ACTIVE and INACTIVE
     * @return A FTCLib COmmand.
     */
    public Command toggle() {
        return new ToggleWrist(this);
    }

    /**
     * Command that toggles the position of the wrist between two set positions
     * following the FTCLib command paradigm.
     * @see // TODO: Add explanatory link here
     */
    public static class ToggleWrist extends CommandBase {
        private final Wrist wrist;

        public ToggleWrist(Wrist subsystem) {
            this.wrist = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            wrist.leftServo.setPosition(!wrist.isActive ? WristState.ACTIVE.position : WristState.INACTIVE.position);
            wrist.rightServo.setPosition(!wrist.isActive ? WristState.ACTIVE.position : WristState.INACTIVE.position);
            wrist.isActive = !wrist.isActive;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    /**
     * Command that sets the position of the wrist to a given position
     * following the FTCLib command paradigm.
     * @see @ // TODO: Add explanatory link here
     */
    public static class SetWristPosition extends CommandBase {
        private final Wrist wrist;
        private final double position;

        public SetWristPosition(WristState state, Wrist subsystem) {
            this.wrist = subsystem;
            position = state.position;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            wrist.leftServo.setPosition(position);
            wrist.rightServo.setPosition(position);
            wrist.isActive = true; // fixme may cause wrist problems?
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}
