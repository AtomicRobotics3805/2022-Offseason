package org.firstinspires.ftc.teamcode.main.other

import org.firstinspires.ftc.teamcode.commandFramework.controls.Controls

/**
 * This class manages the controls for TeleOp OpModes. It's currently just a skeleton of what it
 * would be during a competition, since there are no real commands to register. If you want to
 * register a command, type a line into registerCommands with the following format:
 * gamepad.button.command = Subsystem.command
 * For example, that could look like this:
 * gamepad1.a.startCommand = Lift.toHigh
 * If you used the line above, then whenever you pressed the a button on gamepad1 it would move the
 * lift to the high position.
 */
@Suppress("unused")
object Controls : Controls() {

    /**
     * Registers commands on the gamepads. Currently empty since this class is a skeleton.
     * Directions for registering commands are in the class docs.
     */
    override fun registerCommands() {

    }
}