package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

/**
 * Example placeholder subsystem to represent viper slide/arm system
 */
public class Slide implements Subsystem {

    /**
     * A simple command that extends viper slides with the
     * {@link Slide} Subsystem.  Written explicitly for
     * pedagogical purposes.
     */
    public class Extend extends CommandBase {
        @Override
        public void initialize() {
            // Arm extension logic
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

}





