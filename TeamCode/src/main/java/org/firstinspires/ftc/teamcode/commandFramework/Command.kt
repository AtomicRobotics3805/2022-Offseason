package org.firstinspires.ftc.teamcode.commandFramework

import org.firstinspires.ftc.teamcode.commandFramework.Constants

@Suppress("PropertyName")
abstract class Command {
    var isDone = false
        get() = field || _isDone
    open val _isDone = false
    var isStarted = false

    open fun execute() { }
    open fun start() { }
    open fun end(interrupted: Boolean) { }
}