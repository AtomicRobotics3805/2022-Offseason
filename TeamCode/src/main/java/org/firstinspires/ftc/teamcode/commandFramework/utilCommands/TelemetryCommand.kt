package org.firstinspires.ftc.teamcode.commandFramework.utilCommands

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.commandFramework.Command

@SuppressWarnings("unused")
class TelemetryCommand(private val time: Double, private val message: () -> String) : Command() {

    val timer = ElapsedTime()
    override val _isDone: Boolean
        get() = timer.seconds() > time

    constructor(time: Double, caption: String, data: () -> String) :
            this(time, { caption + ": " + data.invoke() })
    constructor(time: Double, message: String) :
            this(time, { message })
    constructor(time: Double, caption: String, data: String) :
            this(time, { "$caption: $data" })

    override fun start() {
        timer.reset()
    }

    override fun execute() {
        Constants.opMode.telemetry.addLine(message.invoke())
    }
}