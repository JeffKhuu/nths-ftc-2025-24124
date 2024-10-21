package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.BetterRobotCentricDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.utilities.pathfinding.Pathfinder;

public abstract class AutoInstructions {
    LinearOpMode opMode;

    DriveTrain driveTrain;
    Claw claw;
    Slide slides;
    Pathfinder pathfinder;

    public AutoInstructions(LinearOpMode opMode) {
        this.opMode = opMode;

        driveTrain = new BetterRobotCentricDriveTrain(opMode.hardwareMap, getPose());
        claw = new Claw(opMode.hardwareMap);
        slides = new Slide(opMode.hardwareMap);
        pathfinder = new Pathfinder();

        opMode.telemetry.addData("Systems", "Initialized");
        opMode.telemetry.addData("Status", "Initialized");
        opMode.telemetry.update();
    }

    public abstract void execute();

    public abstract Pose2d getPose();
}
