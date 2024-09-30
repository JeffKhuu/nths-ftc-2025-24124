package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

public class RobotCentricDriveTrain extends DriveTrain {
    public RobotCentricDriveTrain(HardwareMap hardwareMap, Pose2d pose2d) {
        super(hardwareMap, pose2d);
    }

    @Override
    public void move(double x, double y, double turn) {
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
    }
}
