package org.firstinspires.ftc.teamcode.commandFramework

class CommandScheduler {

    private val runningCommands = mutableListOf<Command>()
    private val commandsToSchedule = mutableListOf<Command>()
    private val commandsToCancel = mutableMapOf<Command, Boolean>()
    private val gamepads = mutableListOf<CommandGamepad>()
    private val subsystems = mutableListOf<Subsystem>()

    // exercise is healthy (and fun!)
    fun run() {
        updateGamepads()
        updateSubsystems()
        scheduleCommands()
        cancelCommands()
        for (command in commands) {
            command.execute()
            if (command.isDone) {
                commandsToCancel += Pair(command, false)
            }
        }
    }

    fun scheduleCommands() {
        for(command in commandsToSchedule) {
            initCommand(command)
        }
        commandsToSchedule.clear()
    }

    fun cancelCommands() {
        for(pair in commandsToCancel) {
            cancel(pair.key, pair.value)
        }
        commandsToCancel.clear()
    }

    fun initCommand(command: Command) {
        command.start()
        runningCommands += command
    }

    private fun cancel(command: Command, interrupted: Boolean = false) {
        command.end(interrupted)
        runningCommands -= command
    }

    fun cancelAll() {
        for (command in runningCommands) {
            commandsToCancel += Pair(command, true)
        }
        cancelCommands()
        commandsToSchedule.clear()
    }
    
    fun updateGamepads() {
        for (gamepad in gamepads)
            gamepad.update()
    }

    fun updateSubsystems() {
        for (subsystem in subsystems) {
            subsystem.periodic()
            if (findCommand({ it.requirements.contains(subsystem) }) != null)
                subsystem.inUsePeriodic()
        }
    }
    
    fun findCommand(check: (Command) -> Boolean) = findCommands(check).firstOrNull()
    
    fun findCommands(check: (Command) -> Boolean): List<Command> {
        val foundCommands = mutableListOf<Command>()
        for (command in runningCommands) {
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