package org.firstinspires.ftc.teamcode.main.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.Constants.opMode
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.MoveServo
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object BucketLock : Subsystem {
    enum class Position {
        OPEN,
        CLOSED
    }

    @JvmField
    var SPEED = 1.0
    @JvmField
    var NAME = "lock"
    @JvmField
    var OPEN_POSITION = 0.45
    @JvmField
    var CLOSE_POSITION = 0.1

    var position = Position.OPEN

    val open: Command
        get() = MoveServo(latchServo, OPEN_POSITION, SPEED, listOf(this))
    val close: Command
        get() = MoveServo(latchServo, CLOSE_POSITION, SPEED, listOf(this))

    private lateinit var latchServo: Servo

    override fun initialize() {
        latchServo = opMode.hardwareMap.get(Servo::class.java, NAME)
    }
}