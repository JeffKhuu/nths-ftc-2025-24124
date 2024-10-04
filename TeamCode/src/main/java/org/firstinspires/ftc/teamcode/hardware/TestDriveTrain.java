package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.utilities.CarouselSelect;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;

public class TestDriveTrain {
    public final CarouselSelect<Double> speeds = new CarouselSelect<>(
            new Double[]{1.0, 0.5, 0.25} // Speed multipliers
    );

    public double botHeading; // Angle (in radians) the robot is facing
    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final DcMotor rightFront;
    private final DcMotor rightBack;

    public TestDriveTrain(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void move(double x, double y, double turn){
        double leftPower = y + x + turn;
        double leftBackPower = y - x + turn;
        double rightPower = y - x - turn;
        double rightBackPower = y + x + turn;

        double maxPower = Math.max(Math.max(Math.abs(leftPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightPower), Math.abs(rightBackPower)));

        if (maxPower > 1.0) {
            leftPower /= maxPower;
            leftBackPower /= maxPower;
            rightPower /= maxPower;
            rightBackPower /= maxPower;
        }

        setDrivePower(leftPower * speeds.getSelected(),
                leftBackPower * speeds.getSelected(),
                rightPower * speeds.getSelected(),
                rightBackPower * speeds.getSelected());
    };

    /**
     * The method used to move the drive train given four powers for each of the motors.
     *
     * @param leftPower      The power to be given to the front-left motor.
     * @param leftRearPower  The power to be given to the back-left motor.
     * @param rightPower     The power to be given to the front-right motor.
     * @param rightRearPower The power to be given to the back-right motor.
     * @see <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">Mecanum Wheel Guide</a>
     */
    public void setDrivePower(double leftPower, double leftRearPower, double rightPower, double rightRearPower) {
        leftFront.setPower(leftPower);
        leftBack.setPower(leftRearPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightRearPower);
    }
}
