package org.firstinspires.ftc.teamcode.commandFramework

import com.qualcomm.robotcore.util.ElapsedTime

open class CustomCommand(
    private val getDone: () -> Boolean = { true },
    private val _execute: () -> Unit = { },
    private val _start: () -> Unit = { },
    private val _done: (interrupted: Boolean) -> Unit = { },
    private val endTime: Double = 0.0
): Command() {

    override val _isDone: Boolean
        get() = getDone.invoke() && timer.seconds() > endTime
    private val timer = ElapsedTime()

    override fun start() {
        timer.reset()
        _start.invoke()
    }

    override fun execute() {
        _execute.invoke()
    }

    override fun end(interrupted: Boolean) {
        _done.invoke(interrupted)
    }
}