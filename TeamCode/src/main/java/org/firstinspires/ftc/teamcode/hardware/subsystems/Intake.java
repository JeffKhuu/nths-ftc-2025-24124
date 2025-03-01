package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.CommandAction;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

/**
 * @deprecated Replaced with {@link Claw}
 */
@Config
public class Intake extends SubsystemBase implements TelemetrySubject {
    public static class Config {
        // Device name to retrieve from hardwareMap
        public static final String LEFT_INTAKE = "leftIntake";
        public static final String RIGHT_INTAKE = "rightIntake";
    }

    public enum IntakeState { // Accepts values between 0.5 and 1.0
        INTAKE(0.8),
        EXHAUST(-0.8),
        DISABLED(0);

        public final double power;

        IntakeState(double power) {
            this.power = power;
        }
    }

    private CRServo leftIntake;
    private CRServo rightIntake;

    public Intake(HardwareMap hardwareMap) {
        leftIntake = hardwareMap.get(CRServo.class, Config.LEFT_INTAKE);
        rightIntake = hardwareMap.get(CRServo.class, Config.RIGHT_INTAKE);

        leftIntake.setDirection(CRServo.Direction.FORWARD);
        rightIntake.setDirection(CRServo.Direction.REVERSE);

        leftIntake.setPower(IntakeState.DISABLED.power);
        rightIntake.setPower(IntakeState.DISABLED.power);
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {

    }

    /**
     * Move the intake servos to a valid given IntakeState
     *
     * @param state A valid IntakeState
     * @return A RoadRunner Action.
     */
    public Action rotate(IntakeState state) {
        return new CommandAction(setPower(state));
    }

    /**
     * Move the intake servos to a valid given IntakeState
     *
     * @param state A valid IntakeState
     * @param time  Time to spin for
     * @return A RoadRunner Action.
     */
    public Action rotateForSeconds(IntakeState state, int time) {
        return new SequentialAction(
                new CommandAction(setPower(state)),
                new SleepAction(time),
                new CommandAction(setPower(IntakeState.DISABLED))
        );
    }

    /**
     * Set the power of the intake servos to a given IntakeState
     *
     * @param state A valid IntakeState
     * @return An FTCLib Command
     */
    public Command setPower(Intake.IntakeState state) {
        return new SetPower(state, this);
    }

    /**
     * Set the power of the intake servos to a given power
     *
     * @param power An integer
     * @return An FTCLib Command
     */
    public Command setPower(int power) {
        return new SetPower(power, this);
    }

    /**
     * FTCLib command that sets the power of both intake servos
     * Uses the {@link Intake} Subsystem
     */
    public static class SetPower extends CommandBase {
        private final Intake intake;
        private final double position;

        public SetPower(Intake.IntakeState state, Intake subsystem) {
            intake = subsystem;
            position = state.power;
            addRequirements(intake);
        }

        public SetPower(double position, Intake subsystem) {
            intake = subsystem;
            this.position = position;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            intake.setPowers(position);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    /**
     * Set the power of both servos
     *
     * @param power Power to give to the servos
     */
    public void setPowers(double power) {
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }
}
