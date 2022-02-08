package org.firstinspires.ftc.teamcode.commandFramework

class CommandScheduler {
    val runningCommands = mutableListOf<Command>()
    val commandsToSchedule = mutableListOf<Command>()
    val commandsToCancel = mutableMapOf<Command, Boolean>()

    val initActions = mutableListOf<(Command) -> Unit>()
    val executeActions = mutableListOf<(Command) -> Unit>()
    val interruptActions = mutableListOf<(Command) -> Unit>()
    val finishActions = mutableListOf<(Command) -> Unit>()

    fun scheduleCommands() {
        for(command in commandsToSchedule) {
            try {
                initCommand(command)
            } catch (e: Exception) { }
        }
        commandsToSchedule.clear()
    }
    fun cancelCommands() {
        for(pair in commandsToCancel) {
            try {
                cancel(pair.key, pair.value)
            } catch (e: Exception) { }
        }
        commandsToCancel.clear()
    }
    fun initCommand(command: Command) {
        command.start()
        runningCommands += command
        doActions(initActions, command)
    }
    fun cancel(command: Command, interrupted: Boolean = false) {
        command.end(interrupted)
        doActions(finishActions, command)
        runningCommands -= command
    }
    fun cancelAll() {
        for (command in runningCommands) {
            commandsToCancel += Pair(command, true)
        }
        cancelCommands()
        commandsToSchedule.clear()
    }
    fun doActions(actions: List<(Command) -> Unit>, command: Command) {
        for (action in actions)
            action.invoke(command)
    }
}