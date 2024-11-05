package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.CommandAction;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

public class Wrist extends SubsystemBase implements TelemetrySubject {
    public enum WristState {
        HOME(0.72),
        INACTIVE(0.5),
        ACTIVE(0.2);

        final double position;

        WristState(double position) {
            this.position = position;
        }
    }

    public final Servo leftServo;
    public final Servo rightServo;
    public boolean isActive;

    public Wrist(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "leftWrist");
        rightServo = hardwareMap.get(Servo.class, "rightWrist");

        leftServo.setDirection(Servo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.FORWARD);

        leftServo.setPosition(WristState.HOME.position);
        rightServo.setPosition(WristState.HOME.position);
        isActive = true;
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {

    }

    public Action moveTo(WristState state){ // Imperative
        return new CommandAction(moveWrist(state));
    }

    public Command moveWrist(WristState state) { // Declarative
        return new MoveWrist(state, this);
    }

    public Command toggle() {
        return new ToggleWrist(this);
    }

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

    public static class MoveWrist extends CommandBase {
        private final Wrist wrist;
        private final double position;

        public MoveWrist(WristState state, Wrist subsystem) {
            this.wrist = subsystem;
            position = state.position;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            wrist.leftServo.setPosition(position);
            wrist.rightServo.setPosition(position);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}
