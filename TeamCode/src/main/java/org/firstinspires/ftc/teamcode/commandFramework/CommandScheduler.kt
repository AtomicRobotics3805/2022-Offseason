package org.firstinspires.ftc.teamcode.commandFramework

import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem

/**
 * This class runs commands, updates subsystems, and manages Gamepad-related commands. The important
 * functions to use are scheduleCommand(), registerGamepads(), registerSubsystems(), and run().
 */
@Suppress("MemberVisibilityCanBePrivate", "unused")
object CommandScheduler {

    private val runningCommands = mutableListOf<Command>()
    private val commandsToSchedule = mutableListOf<Command>()
    private val commandsToCancel = mutableMapOf<Command, Boolean>()
    private val gamepads = mutableListOf<CustomGamepad>()
    private val subsystems = mutableListOf<Subsystem>()

    /**
     * This function should be run repeatedly every loop. It adds commands if the corresponding
     * Gamepad buttons are being pushed, it runs the periodic functions in Subsystems, it schedules
     * & cancels any commands that need to be started or stopped, and it executes running
     * commands. The reason why it uses a separate function to cancel commands instead of cancelling
     * them itself is because removing items from a list while iterating through that list is a
     * wacky idea.
     */
    // exercise is healthy (and fun!)
    fun run() {
        if (Constants.opMode.opModeIsActive()) {
            updateGamepads()
            updateSubsystems()
            scheduleCommands()
            cancelCommands()
            for (command in runningCommands) {
                command.execute()
                if (command.isDone) {
                    commandsToCancel += Pair(command, false)
                }
            }
        }
        else {
            cancelAll()
        }
    }

    /**
     * Schedules a command. When multiple commands are scheduled, each of them run in parallel.
     * @param command the command to be scheduled
     */
    fun scheduleCommand(command: Command) {
        commandsToSchedule += command
    }

    /**
     * Registers one or more subsystems. This list is used to run the subsystems' periodic
     * functions.
     * @param subsystems the subsystems to be registered
     */
    fun registerSubsystems(vararg subsystems: Subsystem) {
        for (subsystem in subsystems) {
            subsystem.initialize()
            this.subsystems += subsystem
        }
    }

    /**
     * Registers one or more CustomGamepads. These gamepads will be scanned every loop to see if
     * any buttons are being pressed, and if so, their corresponding commands will be scheduled.
     * @param gamepads the gamepads to be registered
     */
    fun registerGamepads(vararg gamepads: CustomGamepad) {
        for (gamepad in gamepads)
            this.gamepads += gamepad
    }

    /**
     * Removes every subsystem and gamepad. This function should generally only be used when an
     * OpMode ends.
     */
    fun unregisterAll() {
        subsystems.clear()
        gamepads.clear()
    }

    /**
     * Cancels every command. This function should generally only be used when an OpMode ends.
     */
    fun cancelAll() {
        for (command in runningCommands) {
            commandsToCancel += Pair(command, true)
        }
        cancelCommands()
        commandsToSchedule.clear()
    }

    /**
     * Initializes every command in the commandsToSchedule list.
     */
    private fun scheduleCommands() {
        for(command in commandsToSchedule) {
            initCommand(command)
        }
        commandsToSchedule.clear()
    }

    /**
     * Cancels every command in the commandsToCancel list.
     */
    private fun cancelCommands() {
        for(pair in commandsToCancel) {
            cancel(pair.key, pair.value)
        }
        commandsToCancel.clear()
    }

    /**
     * Initializes a command. This function first scans to find any conflicts (other commands using
     * the same subsystem). It then checks to see if any of those commands are not interruptible. If
     * some of them aren't interruptible, it ends the initialization process and does not schedule
     * the new command. Otherwise, it cancels the conflicts, runs the new command's start function,
     * and adds it to the list of runningCommands.
     * @param command the new command being initialized
     */
    private fun initCommand(command: Command) {
        for (requirement in command.requirements) {
            val conflicts = findCommands({ it.requirements.contains(requirement) }).toMutableList()
            if (conflicts.contains(command)) {
                conflicts -= command
            }
            for (conflict in conflicts)
                if (!conflict.interruptible) {
                    return
                }
            for (conflict in conflicts)
                commandsToCancel += Pair(command, true)
        }
        command.start()
        runningCommands += command
    }

    /**
     * Ends a command and removes it from the runningCommands list.
     * @param command the command being cancelled
     * @param interrupted whether or not that command was interrupted, such as the OpMode is stopped
     *                    prematurely
     */
    private fun cancel(command: Command, interrupted: Boolean = false) {
        command.end(interrupted)
        runningCommands -= command
    }

    /**
     * Runs the update function for each gamepad.
     */
    private fun updateGamepads() {
        for (gamepad in gamepads)
            gamepad.update()
    }

    /**
     * Runs the periodic function for each subsystem and the inUsePeriodic function for each
     * subsystem being used by at least one command.
     */
    private fun updateSubsystems() {
        for (subsystem in subsystems) {
            subsystem.periodic()
            if (findCommand({ it.requirements.contains(subsystem) }) != null)
                subsystem.inUsePeriodic()
        }
    }

    /**
     * Calls the findCommands() function and uses the first result, or null if there are none
     * @param check the lambda used to determine what kind of command should be found
     * @param commands the list of commands to scan, uses runningCommands by default
     */
    private fun findCommand(check: (Command) -> Boolean, commands : List<Command> = runningCommands) =
        findCommands(check, commands).firstOrNull()

    /**
     * Returns a list of every command in the given list that passes a check. Also scans
     * CommandGroups by recursively calling itself.
     * @param check the lambda used to determine what kind of commands should be found
     * @param commands the list of commands to scan, uses runningCommands by default
     */
    private fun findCommands(check: (Command) -> Boolean, commands : List<Command> = runningCommands):
            List<Command> {
        val foundCommands = mutableListOf<Command>()
        for (command in commands) {
            if (check.invoke(command))
                foundCommands.add(command)
            if (command is CommandGroup) {
                val c = findCommand(check, command.commands)
                if (c != null) foundCommands.add(c)
            }
        }
        return foundCommands
    }
}