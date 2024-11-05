package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;

@AutonomousEx(preload = 0, cycles = 0)
public class RedLeftAuto extends AutoInstructions {

    // Instantiate subsystems
    Pose2d startPose = new Pose2d(new Vector2d(24, -72), 90); //fixme inaccurate pose

    RedLeftAuto(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void stop() {
        FieldConstants.savePose(driveTrain.mecanumDrive.pose);
    }

    @Override
    public Pose2d getStartPose() {
        return startPose;
    }
}
