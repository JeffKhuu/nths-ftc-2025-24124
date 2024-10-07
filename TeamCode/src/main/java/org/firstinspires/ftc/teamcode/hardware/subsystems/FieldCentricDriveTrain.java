package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

public class FieldCentricDriveTrain extends DriveTrain {

    public FieldCentricDriveTrain(HardwareMap hardwareMap, Pose2d pose2d) {
        super(hardwareMap, pose2d);
    }

    @Override
    public void move(double x, double y, double turn) {
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
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
}
