package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.CarouselSelect;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

/**
 * Example placeholder subsystem to represent viper slide/arm system
 */
public class Slide extends SubsystemBase implements TelemetrySubject {

    public final DcMotorEx leftSlide;
    //DcMotorEx rightSlide = null;

    public enum SlideState {
        HOME(0),
        LOW_RUNG(0),
        LOW_BUCKET(0),
        HIGH_RUNG(0),
        HIGH_BUCKET(10250);

        final int position;

        SlideState(int position) {
            this.position = position;
        }
    }

    private static final double SPEED = 0.75;

    private static final int MIN = SlideState.HOME.position;
    private static final int MAX = SlideState.HIGH_BUCKET.position;

    public final CarouselSelect<SlideState> positions = new CarouselSelect<>(
            new Slide.SlideState[]{Slide.SlideState.HOME, Slide.SlideState.HIGH_BUCKET}
    );


    public Slide(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        //rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        // MAKE SURE THE DIRECTIONS ARE SET PROPERLY
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset Encoder Positions
        leftSlide.setTargetPosition(SlideState.HOME.position);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(SPEED);

        // Make sure the bottom is always zero
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setPower(0);
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {
        telemetry.print("⎯⎯⎯⎯⎯⎯⎯⎯SLIDES⎯⎯⎯⎯⎯⎯⎯⎯");
        telemetry.printCarousel(positions);
        telemetry.print("Left Position", leftSlide.getCurrentPosition());
    }

    @Override
    public void periodic() {
        // Enforce limits on the slides
        if (leftSlide.getCurrentPosition() > MAX) {
            leftSlide.setTargetPosition(MAX);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(Slide.SPEED);
        } else if (leftSlide.getCurrentPosition() < MIN) {
            leftSlide.setTargetPosition(MIN);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(Slide.SPEED);
        }
    }


    public Move extend() {
        return new Move(this, SPEED);
    }

    public Move retract() {
        return new Move(this, -SPEED);
    }

    public MoveToPosition moveTo(SlideState state) {
        return new MoveToPosition(this, state.position);
    }

    public MoveToPosition moveTo(int position) {
        return new MoveToPosition(this, position);
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
            if (slideSubsystem.leftSlide.getCurrentPosition() <= MAX && // Enforce limits
                    slideSubsystem.leftSlide.getCurrentPosition() >= MIN) {
                slideSubsystem.leftSlide.setPower(power);
            }
        }

        // Run once after the command is unscheduled
        @Override
        public void end(boolean interrupted) {
            slideSubsystem.setZeroPower();
        }

    }

    /**
     * A simple command that uses motor encoders to drive viper slides to a specified position. Uses the {@link Slide} Subsystem.
     */
    public static class MoveToPosition extends CommandBase {
        private final Slide slideSubsystem;
        private final int encoderPos;

        MoveToPosition(Slide subsystem, int position) {
            slideSubsystem = subsystem;
            encoderPos = position;
            addRequirements(slideSubsystem);
        }

        @Override
        public void initialize() {
            slideSubsystem.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideSubsystem.leftSlide.setTargetPosition(encoderPos);
            slideSubsystem.leftSlide.setPower(Slide.SPEED);
        }
    }

}







