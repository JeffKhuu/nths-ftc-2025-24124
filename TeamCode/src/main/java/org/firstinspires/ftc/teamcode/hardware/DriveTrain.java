package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.constants.FieldConstants.lastSavedPose;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.utilities.ActionCommand;
import org.firstinspires.ftc.teamcode.utilities.CarouselSelect;
import org.firstinspires.ftc.teamcode.utilities.pathfinding.Field;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * Foundational Mecanum Drive Train abstract class. Requires a move method and constructor.
 * Makes use of {@link MecanumDrive} from RoadRunner.
 */
public abstract class DriveTrain extends SubsystemBase implements TelemetrySubject {
    public final MecanumDrive mecanumDrive; // Roadrunner-Based mecanum drive
    public final CarouselSelect<Double> speeds = new CarouselSelect<>(
            new Double[]{1.0, 0.5, 0.25} // Speed multipliers
    );
    Pose2d pose = new Pose2d(0,0,0);

    public double botHeading; // Angle (in radians) the robot is facing

    public DriveTrain(HardwareMap hardwareMap, Pose2d pose2d) {
        mecanumDrive = new MecanumDrive(hardwareMap, pose2d);
        botHeading = mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
        botHeading = mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);// Update Bot Heading
        mecanumDrive.updatePoseEstimate();
    }

    public abstract void move(double x, double y, double turn);

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
        mecanumDrive.leftFront.setPower(leftPower);
        mecanumDrive.leftBack.setPower(leftRearPower);
        mecanumDrive.rightFront.setPower(rightPower);
        mecanumDrive.rightBack.setPower(rightRearPower);
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
        mecanumDrive.leftFront.setPower(powers[0]);
        mecanumDrive.leftBack.setPower(powers[1]);
        mecanumDrive.rightFront.setPower(powers[2]);
        mecanumDrive.rightBack.setPower(powers[3]);
    }

    /**
     * Reset the current heading of the drive train.
     */
    public void resetHeading() {
        mecanumDrive.lazyImu.get().resetYaw();
    }

    @Override
    public void updateTelemetry(TelemetryEx telemetry) {
        telemetry.print("Speed", speeds.getSelected());
        telemetry.print("Heading", botHeading);
    }

    /**
     * Simple command to turn a given number of degrees using the RoadRunner drive train
     *
     * @param angle Angle in Degrees
     * @return Returns a FTC-Lib compatible command using RoadRunner actions to turn a certain number of degrees
     */
    public ActionCommand turn(int angle) {
        Set<Subsystem> requirements = new HashSet<>(Collections.singletonList(this));

        return new ActionCommand(
                mecanumDrive.actionBuilder(mecanumDrive.pose)
                        .turn(Math.toRadians(angle))
                        .build(),
                requirements
        );
    }

    public Action strafeTo(int x, int y){
        return mecanumDrive.actionBuilder(lastSavedPose) //fixme may cause problems
                .strafeTo(new Vector2d(x, y))
                .build();

    }

    /**
     * Turn the robot using the RoadRunner drive train
     * @param angle Angle in Degrees
     * @return RoadRunner Action
     */
    public Action turnTo(int angle){
        return mecanumDrive.actionBuilder(lastSavedPose) //fixme may cause problems
                .turnTo(Math.toRadians(angle))
                .stopAndAdd(() -> FieldConstants.savePose(new Pose2d(FieldConstants.getLastSavedPose().position, Math.toRadians(angle))))
                .build();
    }
}
