package org.firstinspires.ftc.teamcode.main.mechanisms

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.Constants.opMode
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.MoveServo
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.commandFramework.utilCommands.CustomCommand

object Bucket : Subsystem {
    enum class Position {
        COLLECT,
        DOWN
    }

    @JvmField
    var SPEED = 1.0
    @JvmField
    var NAME = "bucket_tilt"
    @JvmField
    var DROP_POSITION = 0.15
    @JvmField
    var COLLECT_POSITION = 0.85

    var currentPosition = Position.COLLECT

    val drop: Command
        get() = CustomCommand( _start = {
            MoveServo(tiltServo, DROP_POSITION, SPEED, listOf(this))
            currentPosition = Position.DOWN
        })
    val collect: Command
        get() = CustomCommand( _start = {
            MoveServo(tiltServo, COLLECT_POSITION, SPEED, listOf(this))
            currentPosition = Position.COLLECT
        })

    private lateinit var tiltServo: Servo

    override fun initialize() {
        tiltServo = opMode.hardwareMap.get(Servo::class.java, NAME)
    }
}