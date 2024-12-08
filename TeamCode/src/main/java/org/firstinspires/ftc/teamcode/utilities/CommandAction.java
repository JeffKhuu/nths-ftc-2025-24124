package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

/**
 * FTCLib Command Wrapper class to convert FROM FTCLib Commands TO RoadRunner actions
 */
public class CommandAction implements Action {
    private final Command command;
    private boolean initialized = false;
    public CommandAction(Command command) {
        this.command = command;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        final boolean initialized = this.initialized;
        if (!initialized) {
            command.schedule();
            this.initialized = true;
        }

        final boolean finished = initialized && !CommandScheduler.getInstance().isScheduled(command);
        if (finished) {
            this.initialized = false;
        }
        return finished;
    }
}
