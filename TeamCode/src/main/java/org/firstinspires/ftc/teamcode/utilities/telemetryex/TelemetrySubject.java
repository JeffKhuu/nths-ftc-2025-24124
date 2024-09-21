package org.firstinspires.ftc.teamcode.utilities.telemetryex;

/**
 * Interface to enable regular Telemetry updates in subsystems
 * used in conjuction with TelemetryMaster
 */
public interface TelemetrySubject {
    /**
     * Method called by a TelemetryMaster to update Telemetry.
     * @param telemetry Extended Telemetry system to have data printed to.
     */
    void updateTelemetry(TelemetryEx telemetry);
}
