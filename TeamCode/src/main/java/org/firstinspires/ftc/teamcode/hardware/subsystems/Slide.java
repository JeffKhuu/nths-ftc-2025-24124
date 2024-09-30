package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

/**
 * Example placeholder subsystem to represent viper slide/arm system
 */
public class Slide extends SubsystemBase implements TelemetrySubject {

    private final DcMotorEx leftSlide;
    //DcMotorEx rightSlide = null;

    public enum SlideState {
        HOME(0),
        LOW_RUNG(0),
        LOW_BUCKET(0),
        HIGH_RUNG(0),
        HIGH_BUCKET(4125);

        final int position;

        SlideState(int position) {
            this.position = position;
        }
    }

    private final double SPEED = 0.75;


    public Slide(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        //rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        // MAKE SURE THE DIRECTIONS ARE SET PROPERLY
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset Encoder Positions
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reset the left slide to the bottom
        leftSlide.setTargetPosition(0);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setTargetPosition(SlideState.HOME.position);
        leftSlide.setPower(SPEED);


        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {

    }

    @Override
    public void periodic() {
        super.periodic();
    }


    public Move extend() {
        return new Move(this, SPEED);
    }

    public Move retract() {
        return new Move(this, -SPEED);
    }

    public MoveToPosition moveToPosition(SlideState state) {
        return new MoveToPosition(this, state);
    }

    public void setZeroPower() {
        leftSlide.setPower(0);
    }


    /**
     * A simple command that moves viper slides with the
     * {@link Slide} Subsystem. Given an amount of power.
     */
    public static class Move extends CommandBase {
        private final Slide slideSubsystem;
        private final double power;

        public Move(Slide subsystem, double power) {
            slideSubsystem = subsystem;
            this.power = power;
            addRequirements(slideSubsystem);
        }

        // Run once when the command is scheduled
        @Override
        public void initialize() {
            slideSubsystem.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        @Override
        public void execute() {
            slideSubsystem.leftSlide.setPower(power);
        }

        // Run once after the command is unscheduled
        @Override
        public void end(boolean interrupted) {
            slideSubsystem.leftSlide.setPower(0);
        }

    }

    /**
     * A simple command that uses motor encoders to drive viper slides to a specified position. Uses the {@link Slide} Subsystem.
     */
    public static class MoveToPosition extends CommandBase {
        private final Slide slideSubsystem;
        private final int encoderPos;

        MoveToPosition(Slide subsystem, SlideState state) {
            slideSubsystem = subsystem;
            encoderPos = state.position;
            addRequirements(slideSubsystem);
        }

        @Override
        public void initialize() {
            slideSubsystem.leftSlide.setTargetPosition(encoderPos);
            slideSubsystem.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideSubsystem.leftSlide.setPower(slideSubsystem.SPEED);
        }
    }

}







