package org.firstinspires.ftc.teamcode.commandFramework.utilCommands

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.commandFramework.Command

@SuppressWarnings("unused")
class Delay(private val time: Double): Command() {
    override val _isDone: Boolean
        get() = timer.seconds() > time || time == 0.0

    private val timer = ElapsedTime()

    override fun start() {
        timer.reset()
    }
}