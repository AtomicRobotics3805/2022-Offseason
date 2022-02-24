package org.firstinspires.ftc.teamcode.commandFramework.subsystems

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.commandFramework.Command
import kotlin.math.abs

@Suppress("unused")
class MoveServo(private val servo: Servo,
                private val position: Double,
                private val speed: Double,
                override val requirements: List<Subsystem> = arrayListOf(),
                override val interruptible: Boolean = true) : Command() {

    private var positionDif = 0.0
    private val timer = ElapsedTime()
    override val _isDone: Boolean
        get() = timer.seconds() > positionDif * speed

    /**
     * Calculates the difference in position, moves the servo, and resets the timer
     */
    override fun start() {
        positionDif = abs(position - positionDif)
        servo.position = position
        timer.reset()
    }
}
