package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

public class Claw extends SubsystemBase implements TelemetrySubject {
    public Servo claw;
    public enum ClawState {
        OPEN(1.0),
        CLOSED(0.0);

        public final double position;
        ClawState(double position){ // Attach an int to the enum
            this.position = position;
        }
    }

    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(1);
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {

    }

    public MoveClaw moveClaw(ClawState state){
        return new MoveClaw(state, this);
    }

    public MoveClaw moveClaw(Double position){
        return new MoveClaw(position, this);
    }

    /**
     * FTCLib command that moves the claw to a specified claw state or position.
     * Uses the {@link Claw} Subsystem
     */
    public static class MoveClaw extends CommandBase{
        private final Claw claw;
        private final double position;

        public MoveClaw(ClawState state, Claw subsystem){
            claw = subsystem;
            position = state.position;
            addRequirements(claw);
        }

        public MoveClaw(double position, Claw subsystem){
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
