package org.firstinspires.ftc.teamcode.utilities.telemetryex;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utilities.ControllerEx;

import java.util.ArrayList;

public class TelemetryMaster { //TODO: DEPRECATE
    private final TelemetryEx telemetry;
    private final ArrayList<TelemetrySubject> subjects;

    public TelemetryMaster(TelemetryEx telemetry){
        this.telemetry = telemetry;
        subjects = new ArrayList<>();
    }

    /**
     * Subscribes a class to the routine telemetry update call.
     * @param subject A class which implements the TelemetrySubject class
     * @return Returns the TelemetryMaster in order to chain methods
     */
    public TelemetryMaster subscribe(TelemetrySubject subject){
        subjects.add(subject); return this;
    }

    /**
     * Unsubscribes a class from the routine telemetry update
     * @param subject A class which implements the TelemetrySubject class
     */
    public void unsubscribe(TelemetrySubject subject){
        subjects.remove(subject);
    }

    /**
     * Updates telemetry for all subscribed classes and subsystems.
     * Update will add data to telemetry from all subsystems in the order they are subscribed in.
     */
    public void update(){
        subjects.forEach(subject -> subject.updateTelemetry(telemetry));
    }
}
