package org.firstinspires.ftc.teamcode.main.other

import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants.opMode
import org.firstinspires.ftc.teamcode.commandFramework.CustomGamepad
import org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms.Lift
import org.firstinspires.ftc.teamcode.mechanismTesting.LiftMechanism

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
object Controls {

    private val gamepad1 = CustomGamepad(opMode.gamepad1)
    private val gamepad2 = CustomGamepad(opMode.gamepad2)

    /**
     * Registers gamepad1 and gamepad2 with the CommandScheduler so that the CommandScheduler can
     * update them every loop
     */
    fun registerGamepads() {
        CommandScheduler.registerGamepads(gamepad1, gamepad2)
    }

    /**
     * Registers commands on the gamepads. Currently empty since this class is a skeleton.
     * Directions for registering commands are in the class docs.
     */
    fun registerCommands() {
        gamepad1.dpadUp.startCommand = { LiftMechanism.toHigh }
        gamepad1.dpadDown.startCommand= { LiftMechanism.toLow }
    }
}