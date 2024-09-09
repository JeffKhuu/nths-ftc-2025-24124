package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * DEPRECATED DRIVE TRAIN
 * Subsystem class which implements a simple mecanum drive train.
 */
public class DriveTrain {
    private static final double MM_PER_INCH = 25.4;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 100 / MM_PER_INCH; // Value originally in mm, converted to in
    public static double GEAR_RATIO = 1;

    private final DcMotor leftMotor, rightMotor, leftRearMotor, rightRearMotor;

    public DriveTrain(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.dcMotor.get("leftFront");
        rightMotor = hardwareMap.dcMotor.get("rightFront");
        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
        rightRearMotor = hardwareMap.dcMotor.get("rightRear");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
    }


    public static final double FORWARD_SPEED = 0.6; // Forward speed constant
    public static final double TURN_SPEED = 0.5;   // Turn speed constant

    // The constant variable storing an array with the four motor powers to move the robot forwards.
    public static final double[] FORWARD = {FORWARD_SPEED, FORWARD_SPEED, FORWARD_SPEED, FORWARD_SPEED};

    // The constant variable storing an array with the four motor powers to move the robot backwards.
    public static final double[] BACKWARD = {-FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED};

    // The constant variable storing an array with the four motor powers to move the robot left.
    public static final double[] LEFT = {-FORWARD_SPEED, FORWARD_SPEED, FORWARD_SPEED, -FORWARD_SPEED};

    // The constant variable storing an array with the four motor powers to move the robot right.
    public static final double[] RIGHT = {FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED, FORWARD_SPEED};

    // The constant variable storing an array with the four motor powers to spin the robot clockwise.
    public static final double[] SPIN_CW = {TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED};

    // The constant variable storing an array with the four motor powers to spin the robot counter-clockwise.
    public static final double[] SPIN_CCW = {-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED};


    /**
     * The method used to move the drive train given three inputs, most likely from a gamepad.
     *
     * @param x     A Value to dictate the "X" direction of the drive train (+value = forward)
     * @param y     A value to dictate the "Y" direction of the drive train (+value = forward)
     * @param turn  A Value to dictate the rotation of the drive train (+value = clockwise)
     * @param speed Multiplier applied to all calculated power to scale the final power
     */
    public void move(double x, double y, double turn, double speed) {
        //x: left-stick-x
        //y: left-stick-y
        //Turn: right-stick-x

        double leftPower = y + x + turn;
        double leftBackPower = y - x + turn;
        double rightPower = y - x - turn;
        double rightBackPower = y + x - turn;

        double maxPower = Math.max(Math.max(Math.abs(leftPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightPower), Math.abs(rightBackPower)));

        if (maxPower > 1.0) {
            leftPower /= maxPower;
            leftBackPower /= maxPower;
            rightPower /= maxPower;
            rightBackPower /= maxPower;
        }
        setDrivePower(leftPower * speed,
                rightPower * speed,
                leftBackPower * speed,
                rightBackPower * speed);

    }


    /**
     * The method used to move the drive train given four powers for each of the motors.
     *
     * @param leftPower      The power to be given to the front-left motor.
     * @param rightPower     The power to be given to the front-right motor.
     * @param leftRearPower  The power to be given to the back-left motor.
     * @param rightRearPower The power to be given to the back-right motor.
     * @see <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">Mecanum Wheel Guide</a>
     */
    public void setDrivePower(double leftPower, double rightPower, double leftRearPower, double rightRearPower) {
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
     *               Front Left, Front Right, Back Left, Back Right
     * @see <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">Mecanum Wheel Guide</a>
     */
    public void setDrivePower(double[] powers) {
        leftMotor.setPower(powers[0]);
        rightMotor.setPower(powers[1]);
        leftRearMotor.setPower(powers[2]);
        rightRearMotor.setPower(powers[3]);
    }

}
