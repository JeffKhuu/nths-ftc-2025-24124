package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.constants.FieldConstants.START_POSITION;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Extended autonomous annotation used to provide more data than built-in FTC Autonomous annotation
 */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface AutonomousEx {
    int preload(); // Number of preloads successfully executed by an AUTO
    int cycles(); // Number of cycles successfully executed by an AUTO

    START_POSITION position(); // Starting position of the robot
}
