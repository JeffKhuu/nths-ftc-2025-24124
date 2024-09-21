package org.firstinspires.ftc.teamcode.utilities.telemetryex;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Extended Telemetry for ease of processing telemetry.
 */
public class TelemetryEx {
    private final Telemetry telemetry;

    public TelemetryEx(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void print(String caption){
        telemetry.addLine(caption);
    }

    public void print(String header, Object caption){
        telemetry.addData(header, caption);
    }
}
