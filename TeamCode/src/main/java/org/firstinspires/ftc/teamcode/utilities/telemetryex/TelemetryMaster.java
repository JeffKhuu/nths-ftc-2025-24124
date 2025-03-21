package org.firstinspires.ftc.teamcode.utilities.telemetryex;

import com.sun.tools.javac.util.StringUtils;

import org.firstinspires.ftc.teamcode.utilities.Utilities;

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
        telemetry.print(subject.getClass().getSimpleName().toUpperCase() + " subscribed to telemetry.");
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
        subjects.forEach(subject -> {
            String subsystemTitle = Utilities.center(40, subject.getClass().getSimpleName().toUpperCase());

            telemetry.print(subsystemTitle); // Print the subsystem's title
            subject.updateTelemetry(telemetry); // Call on the subsystem to report telemetry
            telemetry.print(""); // Print a line break
        });
    }
}
