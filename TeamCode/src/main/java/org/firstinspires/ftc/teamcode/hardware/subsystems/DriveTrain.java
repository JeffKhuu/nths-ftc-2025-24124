package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utilities.System;

/**
 * Subsystem class which implements a simple mecanum drive train, IMU readings and odometry
 * to assist in robot movement and driving.
 */
public class DriveTrain extends System {
    private static final double MM_PER_INCH = 25.4;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 100 / MM_PER_INCH; // Value originally in mm, converted to in
    public static double GEAR_RATIO = 1;

    private DcMotor leftMotor, rightMotor, leftRearMotor, rightRearMotor;

    public DriveTrain(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
    }
}
