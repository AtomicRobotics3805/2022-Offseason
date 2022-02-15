package org.firstinspires.ftc.teamcode.commandFramework

import com.qualcomm.robotcore.util.ElapsedTime

open class CustomCommand(
    private val getDone: () -> Boolean = { true },
    private val _execute: () -> Unit = { },
    private val _start: () -> Unit = { },
    private val _done: (interrupted: Boolean) -> Unit = { },
    private val endTime: Double = -1.0
): Command() {
    private val timer = ElapsedTime()

    override val _isDone: Boolean
        get() = getDone.invoke() && (timer.seconds() > endTime || endTime >= 0)

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