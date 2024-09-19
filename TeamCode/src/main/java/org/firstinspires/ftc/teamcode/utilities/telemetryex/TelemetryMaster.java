package org.firstinspires.ftc.teamcode.utilities.telemetryex;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utilities.ControllerEx;

import java.util.ArrayList;

public class TelemetryMaster {
    private final TelemetryEx telemetry;
    private final ArrayList<TelemetrySubject> subjects;

    public TelemetryMaster(TelemetryEx telemetry){
        this.telemetry = telemetry;
        subjects = new ArrayList<>();
    }

    public TelemetryMaster subscribe(TelemetrySubject subject){
        subjects.add(subject); return this;
    }

    public void unsubscribe(TelemetrySubject subject){
        subjects.remove(subject);
    }

    public void update(){
        subjects.forEach(subject -> subject.updateTelemetry(telemetry));
    }
}
