package org.firstinspires.ftc.teamcode.commandFramework.utilCommands

import org.firstinspires.ftc.teamcode.commandFramework.Command

class WaitUntil(private val check: () -> Boolean) : Command() {

    override val _isDone: Boolean
        get() = check.invoke()
}