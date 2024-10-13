package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

public class BetterRobotCentricDriveTrain extends DriveTrain {
    public BetterRobotCentricDriveTrain(HardwareMap hardwareMap, Pose2d pose2d) {
        super(hardwareMap, pose2d);
    }

    @Override
    public void move(double x, double y, double turn) {
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftPower = power * cos / max + turn;
        double leftBackPower = power * sin / max + turn;
        double rightPower = power * sin / max - turn;
        double rightBackPower = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftPower /= power + Math.abs(turn);
            leftBackPower /= power + Math.abs(turn);
            rightPower /= power + Math.abs(turn);
            rightBackPower /= power + Math.abs(turn);
        }

        setDrivePower(leftPower * speeds.getSelected(),
                leftBackPower * speeds.getSelected(),
                rightPower * speeds.getSelected(),
                rightBackPower * speeds.getSelected());
    }
}
