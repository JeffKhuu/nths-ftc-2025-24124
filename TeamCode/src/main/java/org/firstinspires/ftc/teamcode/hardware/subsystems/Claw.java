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

public class Claw extends SubsystemBase implements TelemetrySubject {
    public Servo claw;
    public boolean isClawOpen;

    public enum ClawState {
        OPEN(0.65),
        CLOSED(0.8);

        public final double position;

        ClawState(double position) {
            this.position = position;
        }
    }

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(ClawState.CLOSED.position);
        isClawOpen = false;
    }

    @Override
    public void periodic() {
        //isOpen = claw.getPosition() == ClawState.OPEN.position;
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {
        telemetry.print("⎯⎯⎯⎯⎯⎯⎯⎯⎯CLAW⎯⎯⎯⎯⎯⎯⎯⎯⎯");
        telemetry.print("Position", claw.getPosition());
    }

    public Action setTo(ClawState state){
        return new CommandAction(moveClaw(state));
    }

    public Command moveClaw(ClawState state) {
        return new MoveClaw(state, this);
    }

    public Command moveClaw(Double position) {
        return new MoveClaw(position, this);
    }

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
