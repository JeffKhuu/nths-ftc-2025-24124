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

    private ControllerEx(Gamepad gamepad) {
        super(gamepad);
    }

    public static ControllerBuilder Builder(Gamepad gamepad) {
        return new ControllerBuilder(gamepad);
    }

    public static class ControllerBuilder {

        private final ControllerEx controller;

        public ControllerBuilder(Gamepad gamepad) {
            controller = new ControllerEx(gamepad);
        }

        public ControllerBuilder bind(GamepadKeys.Button button, Command command) {
            controller.getGamepadButton(button).whenPressed(command);
            return this;
        }
        public ControllerBuilder bind(GamepadKeys.Button button, GamepadKeys.Button button2, Command command) {
            controller.getGamepadButton(button)
                    .and(controller.getGamepadButton(button2))
                    .whenActive(command);
            return this;
        }

        public ControllerBuilder toggle(GamepadKeys.Button button, GamepadKeys.Button button2, Command command) {
            controller.getGamepadButton(button)
                    .and(controller.getGamepadButton(button2))
                    .toggleWhenActive(command);
            return this;
        }

        public ControllerBuilder bindWhileHeld(GamepadKeys.Button button, Command command) {
            controller.getGamepadButton(button).whileHeld(command);
            return this;
        }

        public ControllerBuilder bindWhenHeld(GamepadKeys.Button button, Command command) {
            controller.getGamepadButton(button).whenHeld(command);
            return this;
        }

        public ControllerBuilder bindWhenReleased(GamepadKeys.Button button, Command command) {
            controller.getGamepadButton(button).whenReleased(command);
            return this;
        }



        public ControllerEx build() {
            return controller;
        }
    }
}
