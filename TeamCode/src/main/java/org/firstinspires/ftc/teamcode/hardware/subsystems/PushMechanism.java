package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.ActionCommand;
import org.firstinspires.ftc.teamcode.utilities.CommandAction;

public class PushMechanism extends SubsystemBase {
    public final String SERVO_NAME = "pusher";

    public enum PushState {
        INACTIVE(0.05),
        ACTIVE(0.33);

        final double position;

        PushState(double position){
            this.position = position;
        }
    }

    Servo pusher;

    public PushMechanism(HardwareMap hardwareMap){
        pusher = hardwareMap.get(Servo.class, SERVO_NAME);
        pusher.setPosition(PushState.INACTIVE.position);
    }
    public Action moveTo(PushState state) { return new CommandAction(setTo(state)); }
    public Command setTo(PushState state){
        return new SetPosition(state, this);
    }
    public Command setTo(double position) { return new SetPosition(position, this); }

    /**
     * FTCLib command that moves the claw to a specified claw state or position.
     * Uses the {@link PushMechanism} Subsystem
     */
    public static class SetPosition extends CommandBase {
        private final PushMechanism pusher;
        private final double position;

        public SetPosition(PushState state, PushMechanism subsystem) {
            pusher = subsystem;
            position = state.position;
            addRequirements(pusher);
        }

        public SetPosition(double position, PushMechanism subsystem) {
            pusher = subsystem;
            this.position = position;
            addRequirements(pusher);
        }

        @Override
        public void initialize() {
            pusher.pusher.setPosition(position);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}
