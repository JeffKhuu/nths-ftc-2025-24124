package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

public class Wrist extends SubsystemBase implements TelemetrySubject {
    public enum WristState {
        HOME(0),
        INACTIVE(0.4),
        ACTIVE(0.8);

        final double position;

        WristState(double position) {
            this.position = position;
        }
    }

    public final Servo leftServo;
    public final Servo rightServo;

    public Wrist(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "leftWrist");
        rightServo = hardwareMap.get(Servo.class, "rightWrist");

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.FORWARD);

        leftServo.setPosition(WristState.HOME.position);
        rightServo.setPosition(WristState.HOME.position);
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {

    }

    public MoveWrist moveWrist(WristState state) {
        return new MoveWrist(state, this);
    }

    public MoveWrist toggleWrist() {
        if (leftServo.getPosition() == WristState.ACTIVE.position) {
            return new MoveWrist(WristState.INACTIVE, this);
        } else {
            return new MoveWrist(WristState.ACTIVE, this);
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
