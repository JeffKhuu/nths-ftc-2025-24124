package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Written by Team 24124
 * An extended gamepad based on FTCLib's GamepadEx. Provides methods for registering commands.
 */
public class ControllerEx extends GamepadEx {
    /**
     * The constructor, that contains the gamepad object from the
     * opmode.
     *
     * @param gamepad the gamepad object from the opmode
     */
    public ControllerEx(Gamepad gamepad) {
        super(gamepad);
    }

    /**
     * Registers four commands to the dpad of the controller.
     * Order of parameters: DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT
     * @param upAction FTCLib command to be ran when the DPAD_UP button is pressed.
     * @param downAction FTCLib command to be ran when the DPAD_DOWN button is pressed.
     */
    public void registerDPad(Command upAction, Command downAction){
        this.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(upAction);
        this.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(downAction);
    }
}
