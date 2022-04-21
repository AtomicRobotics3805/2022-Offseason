package org.firstinspires.ftc.teamcode.commandFramework.utilCommands

import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.controls.ToggleStates
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem

class ToggleCommand(val name: String, vararg val commands: Command) : Command() {

    private lateinit var activeCommand: Command

    override val _isDone: Boolean
        get() = activeCommand.isDone

    override fun start() {
        if (ToggleStates.states[name] == null) {
            ToggleStates.states[name] = 0
        }
        activeCommand = commands[ToggleStates.states[name]!!]
        ToggleStates.states[name] = ToggleStates.states[name]!! + 1
        if (ToggleStates.states[name]!! >= commands.size) {
            ToggleStates.states[name] = 0
        }
        activeCommand.start()
    }

    override fun execute() {
        activeCommand.execute()
    }

    override fun end(interrupted: Boolean) {
        activeCommand.end(interrupted)
    }

    override val requirements: List<Subsystem>
        get() {
            val requirements = ArrayList<Subsystem>()
            commands.forEach { command ->
                command.requirements.forEach {
                    requirements += it
                }
            }
            return requirements
        }

    override val interruptible: Boolean
        get() = activeCommand.interruptible
}