package org.firstinspires.ftc.teamcode.commandFramework

/**
 * This class runs commands, updates subsystems, and manages Gamepad-related commands. The important
 * methods to use are scheduleCommand(), registerGamepads(), registerSubsystems(), and run().
 */
object CommandScheduler {

    private val runningCommands = mutableListOf<Command>()
    private val commandsToSchedule = mutableListOf<Command>()
    private val commandsToCancel = mutableMapOf<Command, Boolean>()
    private val gamepads = mutableListOf<CustomGamepad>()
    private val subsystems = mutableListOf<Subsystem>()

    /**
     * This function should be run repeatedly every loop. It adds commands if the corresponding
     * Gamepad buttons are being pushed, it runs the periodic methods in Subsystems, it schedules
     * & cancels any commands that need to be started or stopped, and it executes running
     * commands. The reason why it uses a separate method to cancel commands instead of cancelling
     * them itself is because removing items from a list while iterating through that list is a
     * wacky idea.
     */
    // exercise is healthy (and fun!)
    fun run() {
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

    fun scheduleCommand(command: Command) {
        commandsToSchedule += command
    }

    fun registerSubsystems(vararg subsystems: Subsystem) {
        for (subsystem in subsystems)
            this.subsystems += subsystem
    }

    fun registerGamepads(vararg gamepads: CustomGamepad) {
        for (gamepad in gamepads)
            this.gamepads += gamepad
    }

    fun cancelAll() {
        for (command in runningCommands) {
            commandsToCancel += Pair(command, true)
        }
        cancelCommands()
        commandsToSchedule.clear()
    }

    private fun scheduleCommands() {
        for(command in commandsToSchedule) {
            initCommand(command)
        }
        commandsToSchedule.clear()
    }

    private fun cancelCommands() {
        for(pair in commandsToCancel) {
            cancel(pair.key, pair.value)
        }
        commandsToCancel.clear()
    }

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

    private fun cancel(command: Command, interrupted: Boolean = false) {
        command.end(interrupted)
        runningCommands -= command
    }

    private fun updateGamepads() {
        for (gamepad in gamepads)
            gamepad.update()
    }

    private fun updateSubsystems() {
        for (subsystem in subsystems) {
            subsystem.periodic()
            if (findCommand({ it.requirements.contains(subsystem) }) != null)
                subsystem.inUsePeriodic()
        }
    }

    private fun findCommand(check: (Command) -> Boolean, commands : List<Command> = runningCommands) =
        findCommands(check, commands).firstOrNull()

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