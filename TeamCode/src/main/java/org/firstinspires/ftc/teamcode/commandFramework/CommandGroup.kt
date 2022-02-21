package org.firstinspires.ftc.teamcode.commandFramework

fun sequential(block: SequentialCommandGroup.() -> Unit): SequentialCommandGroup {
    return SequentialCommandGroup().apply(block)
}

fun parallel(block: ParallelCommandGroup.() -> Unit): ParallelCommandGroup {
    return ParallelCommandGroup().apply(block)
}

abstract class CommandGroup: Command() {

    val commands: MutableList<AtomicCommand> = mutableListOf()
    override val _isDone: Boolean
        get() = commands.isEmpty()

    operator fun plusAssign(command: Command) {
        commands += command
    }

    operator fun AtomicCommand.unaryPlus() = commands.add(this)

    override fun done(interrupted: Boolean) {
        for (command in commands) {
            command.done(interrupted)
        }
    }
}

class SequentialCommandGroup: CommandGroup() {

    override fun start() {
        if (commands.isNotEmpty()) {
            commands[0].start()
        }
    }

    override fun execute() {
        if (commands.isNotEmpty()) {
            if (!commands[0].isStarted) {
                commands[0].start()
                commands[0].isStarted = true
            }
            commands[0].execute()
            if (commands[0].isDone) {
                commands[0].done(false)
                commands.removeFirst()
                if (commands.isNotEmpty())
                    commands[0].start()
            }
        }
    }
}

class ParallelCommandGroup: CommandGroup() {
    private val commandsToCancel: MutableMap<Command, Boolean> = mutableMapOf()

    override fun start() {
        for (command in commands) {
            command.start()
            command.isStarted = true
        }
    }

    override fun execute() {
        for (command in commands) {
            if (!command.isStarted) {
                command.start()
                command.isStarted = true
            }
            command.execute()
            if(command.isDone) {
                commandsToCancel += Pair(command, false)
            }
        }
        clearCommands()
    }

    private fun clearCommands() {
        for (pair in commandsToCancel) {
            cancel(pair.key, pair.value)
        }
        commandsToCancel.clear()
    }

    private fun cancel(command: Command, interrupted: Boolean) {
        command.done(interrupted)
        commands -= command
    }
}
