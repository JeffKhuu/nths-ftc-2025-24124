package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.utilities.selectors.ArraySelect;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetryEx;
import org.firstinspires.ftc.teamcode.utilities.telemetryex.TelemetrySubject;

/**
 * Foundational Mecanum Drive Train abstract class. Requires a move method and constructor.
 * Makes use of {@link MecanumDrive} from RoadRunner.
 *
 * @version 1.0.0
 */
public abstract class DriveTrain extends SubsystemBase implements TelemetrySubject {
    public final MecanumDrive mecanumDrive; // Roadrunner-Based mecanum drive
    private final ArraySelect<Double> speeds = new ArraySelect<>(
            new Double[]{0.5, 1.0} // Speed multipliers
    );

    public double botHeading; // Angle (in radians) the robot is facing

    public DriveTrain(HardwareMap hardwareMap, Pose2d pose2d) {
        FieldConstants.savePose(pose2d);
        mecanumDrive = new MecanumDrive(hardwareMap, pose2d); // Configure motor names in this constructor
        botHeading = mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
        botHeading = mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);// Update Bot Heading
        mecanumDrive.updatePoseEstimate();
    }

    public ArraySelect<Double> getSpeeds() {
        return speeds;
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

        telemetry.print("Position (x)", mecanumDrive.pose.position.x);
        telemetry.print("Position (y)", mecanumDrive.pose.position.y);
    }

    /**
     * Strafe the robot to a set of given coordinates using the RoadRunner drive train
     *
     * @param x x coordinate to strafe to
     * @param y y coordinate to strafe to
     * @return RoadRunner Action
     */
    public Action strafeTo(double x, double y) {
        Pose2d lastPose = FieldConstants.getLastSavedPose();
        FieldConstants.savePose(new Pose2d(x, y, lastPose.heading.toDouble()));

        return mecanumDrive.actionBuilder(lastPose) //fixme may cause problems
                .strafeTo(new Vector2d(x, y))
                .build();
    }

    /**
     * Strafe the robot to a set of given coordinates using the RoadRunner drive train
     *
     * @param x     x coordinate to strafe to
     * @param y     y coordinate to strafe to
     * @param accel Acceleration Constraint
     * @param vel   Velocity Constraint
     * @return RoadRunner Action
     */
    public Action strafeTo(double x, double y, ProfileAccelConstraint accel, VelConstraint vel) {
        Pose2d lastPose = FieldConstants.getLastSavedPose();
        FieldConstants.savePose(new Pose2d(x, y, lastPose.heading.toDouble()));

        return mecanumDrive.actionBuilder(lastPose) //fixme may cause problems
                .strafeTo(new Vector2d(x, y), vel, accel)
                .build();
    }

    /**
     * Strafe the robot to a set of given coordinates using the RoadRunner drive train
     *
     * @param x       x coordinate to strafe to
     * @param y       y coordinate to strafe to
     * @param heading heading to rotate to (in degrees)
     * @return RoadRunner Action
     */
    public Action strafeTo(double x, double y, int heading) {
        Pose2d lastPose = FieldConstants.getLastSavedPose();

        FieldConstants.savePose(new Pose2d(x, y, Math.toRadians(heading)));
        return mecanumDrive.actionBuilder(lastPose) //fixme may cause problems
                .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading), new TranslationalVelConstraint(300))
                .build();


    }

    /**
     * Turn the robot using the RoadRunner drive train
     *
     * @param angle Angle in Degrees
     * @return RoadRunner Action
     */
    public Action turnTo(double angle) {
        Pose2d lastPose = FieldConstants.getLastSavedPose();
        FieldConstants.savePose(new Pose2d(lastPose.position, Math.toRadians(angle)));

        return mecanumDrive.actionBuilder(lastPose) //fixme may cause problems
                .turnTo(Math.toRadians(angle))
                .build();
    }


}
