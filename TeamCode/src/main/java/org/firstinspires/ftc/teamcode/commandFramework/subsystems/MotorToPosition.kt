package org.firstinspires.ftc.teamcode.util.commands.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.commandFramework.Command
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign

/**
 *  MotorToPosition is a class used to rotate a motor to a specific position. It uses proportional powering
 * slow down before reaching the target destination.
 *
 * @param motor the motor to move
 * @param targetPosition where the motor should move to
 * @param speed how fast it should move there
 * @param minError minimum error
 * @param kP multiplied by the error and speed to get the power
 */
@Suppress("MemberVisibilityCanBePrivate")
open class MotorToPosition(protected val motor: DcMotor, protected val targetPosition: Int,
                           protected var speed: Double, protected val minError: Int = 15,
                           protected val kP: Double = 0.005): Command() {

    protected val timer = ElapsedTime()
    protected val positions: MutableList<Double> = mutableListOf()
    protected val savesPerSecond = 10.0
    protected var saveTimes: MutableList<Double> = mutableListOf()
    protected var error: Int = 0
    protected var direction: Double = 0.0
    override val _isDone: Boolean
        get() = abs(error) < minError

    /**
     * Sets the motor's mode to RUN_USING_ENCODER, sets the error to the difference between the target and current
     * positions, and sets the direction to the sign of the error
     */
    override fun start() {
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        error = targetPosition - motor.currentPosition
        direction = sign(error.toDouble())
    }

    /**
     * Updates the error and direction, then calculates and sets the motor power
     */
    override fun execute() {
        error = targetPosition - motor.currentPosition
        direction = sign(error.toDouble())
        val power = kP * abs(error) * speed * direction
        motor.power = Range.clip(power, -min(speed, 1.0), min(speed, 1.0))
        cancelIfStalled()
    }

    /**
     * Stops the motor
     */
    override fun end(interrupted: Boolean) {
        motor.power = 0.0
    }

    fun cancelIfStalled() {
        val lastTime = if (saveTimes.size == 0) 0.0 else saveTimes.last()
                if ((saveTimes.size == 0 && timer.seconds() > 1 / savesPerSecond) ||
            timer.seconds() - saveTimes.last() > 1 / savesPerSecond) {

        }

    }
}