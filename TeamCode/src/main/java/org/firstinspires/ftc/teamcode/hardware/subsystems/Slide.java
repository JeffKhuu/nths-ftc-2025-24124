package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
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

    private final double SPEED = 0.75;

    public Slide(HardwareMap hardwareMap){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        //rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        // MAKE SURE THE DIRECTIONS ARE SET PROPERLY
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset Encoder Positions
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {

    }

    @Override
    public void periodic() {
        super.periodic();
    }


    public Extend extend(){
        return new Extend(this);
    }

    public Retract retract(){
        return new Retract(this);
    }

    /**
     * A simple command that extends viper slides with the
     * {@link Slide} Subsystem.
     */
    public static class Extend extends CommandBase {
        private final Slide slideSubsystem;

        public Extend(Slide subsystem){
            slideSubsystem = subsystem;
            addRequirements(slideSubsystem);
        }

        // Run once when the command is scheduled
        @Override
        public void initialize() {
            slideSubsystem.leftSlide.setPower(slideSubsystem.SPEED);
        }

        // Run once after the command is unscheduled
        @Override
        public void end(boolean interrupted) {
            slideSubsystem.leftSlide.setPower(0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }

    }

    /**
     * A simple command that extends viper slides with the
     * {@link Slide} Subsystem.
     */
    public static class Retract extends CommandBase {
        private final Slide slideSubsystem;

        public Retract(Slide subsystem){
            slideSubsystem = subsystem;
            addRequirements(slideSubsystem);
        }

        // Run once when the command is scheduled
        @Override
        public void initialize() {
            slideSubsystem.leftSlide.setPower(-slideSubsystem.SPEED);
        }

        // Run once after the command is unscheduled
        @Override
        public void end(boolean interrupted) {
            slideSubsystem.leftSlide.setPower(0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }

    }

}





