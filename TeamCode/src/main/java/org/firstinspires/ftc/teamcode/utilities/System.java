package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Abstract class implemented by robot subsystems.
 */
public abstract class System {
    protected HardwareMap hardwareMap;

    /**
     * Constructor used to initialize hardwareMap variable. Required by all subsystems.
     * @param hardwareMap Hardware Map provided by an opMode
     */
    public System(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        init();
    }

    /**
     * Abstract method called during the construction of a child subsystem class.
     */
    public abstract void init();
}
