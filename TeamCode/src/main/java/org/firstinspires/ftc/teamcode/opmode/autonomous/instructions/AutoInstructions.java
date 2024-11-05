package org.firstinspires.ftc.teamcode.opmode.autonomous.instructions;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.utilities.AutonomousEx;
import org.firstinspires.ftc.teamcode.utilities.pathfinding.Pathfinder;

public abstract class AutoInstructions {
    LinearOpMode opMode;
    public DriveTrain driveTrain;
    public Claw claw;
    public Slide slides;
    public Wrist wrist;
    Pathfinder pathfinder;

    public AutoInstructions(LinearOpMode opMode) {
        this.opMode = opMode;
        // Create all subsystems that can be used in autonomous instructions
        // IMPORTANT: Make sure to unregister subsystesms in AutoOpMode.java
        driveTrain = new RobotCentricDriveTrain(opMode.hardwareMap, getStartPose());
        claw = new Claw(opMode.hardwareMap);
        slides = new Slide(opMode.hardwareMap);
        wrist = new Wrist(opMode.hardwareMap);
        pathfinder = new Pathfinder();

        opMode.telemetry.addData("Systems", "Initialized");
        opMode.telemetry.update();
    }

    /**
     * Init is ran BEFORE the START (▶) button is pressed.
     * Preferred use is for setup or to calculate paths.
     */
    public abstract void init();

    /**
     * Execute is ran AFTER the START (▶) button is pressed. Execute is ran only once, not on a loop.
     * Used for the main autonomous instructions.
     */
    public abstract void execute();

    /**
     * Stop is ran after execute has completely finished OR if the stop button is pressed.
     * Used for unregistering subsystems or preparing for teleOp.
     */
    public abstract void stop();

    public void autoInfoTelemetry(Telemetry telemetry){
        AutonomousEx autoData = getAutoData();
        telemetry.addLine();
        telemetry.addLine("⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯");
        telemetry.addData("Preload", autoData.preload());
        telemetry.addData("Cycles", autoData.cycles());
        telemetry.addData("Park", autoData.park());
        telemetry.addLine("⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯");
    }

    public abstract Pose2d getStartPose();
    public AutonomousEx getAutoData(){
        return getClass().getAnnotation(AutonomousEx.class);
    }
}
