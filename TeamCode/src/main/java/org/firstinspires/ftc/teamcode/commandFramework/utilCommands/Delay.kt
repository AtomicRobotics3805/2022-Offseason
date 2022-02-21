package org.firstinspires.ftc.teamcode.commandFramework.utilCommands

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.commandFramework.Command

/**
 * This class is a simple delay that waits for a certain amount of time. Like other delays, it
 * should be put in a sequential command group before the command that needs to be delayed.
 */
@SuppressWarnings("unused")
class Delay(private val time: Double): Command() {
    override val _isDone: Boolean
        get() = timer.seconds() > time

    private val timer = ElapsedTime()

    /**
     * Resets timer
     */
    override fun start() {
        timer.reset()
    }
}