package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utilities.CarouselSelect;
import org.firstinspires.ftc.teamcode.utilities.System;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

public class DriveTrainFieldCentric extends SubsystemBase implements TelemetrySubject {

    private final DcMotor leftMotor, rightMotor, leftRearMotor, rightRearMotor;
    private final IMU imu;

    public final CarouselSelect<Double> speeds = new CarouselSelect<>(
            new Double[]{1.0, 0.5, 0.25} // Speed multipliers
    );

    double botHeading;

    public DriveTrainFieldCentric(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.dcMotor.get("leftFront");
        rightMotor = hardwareMap.dcMotor.get("rightFront");
        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
        rightRearMotor = hardwareMap.dcMotor.get("rightRear");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // Update Bot Heading
    }

    public void move(double x, double y, double turn) {
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading); // Field Centric Drive Train
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1; // Counteract imperfect strafing

        double normalize = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftPower = (rotY + rotX + turn) / normalize;
        double leftBackPower = (rotY - rotX + turn) / normalize;
        double rightPower = (rotY - rotX - turn) / normalize;
        double rightBackPower = (rotY + rotX - turn) / normalize;

        setDrivePower(leftPower * speeds.getSelected(),
                leftBackPower * speeds.getSelected(),
                rightPower * speeds.getSelected(),
                rightBackPower * speeds.getSelected());
    }

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
        leftMotor.setPower(leftPower);
        leftRearMotor.setPower(leftRearPower);
        rightMotor.setPower(rightPower);
        rightRearMotor.setPower(rightRearPower);
    }

    /**
     * The method used to move the drive train given an array containing four powers for each of the motors.
     *
     * @param powers An array of doubles containing four powers to set each motor.
     *               The order in which power is given to the motors is as follows:
     *               Front Left, Back Left, Front Right, Back Right
     * @see <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">Mecanum Wheel Guide</a>
     */
    public void setDrivePower(double[] powers) {
        leftMotor.setPower(powers[0]);
        leftRearMotor.setPower(powers[1]);
        rightMotor.setPower(powers[2]);
        rightRearMotor.setPower(powers[3]);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {
        telemetry.print("Speed", speeds.getSelected());
        telemetry.print("Heading", botHeading);
    }
}
