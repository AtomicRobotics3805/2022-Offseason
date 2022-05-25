package org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem

@Suppress("unused", "MemberVisibilityCanBePrivate")
@Config
object LED : Subsystem {

    @JvmField
    var NAME = "LED"

    private lateinit var led: DigitalChannel

    override fun initialize() {
        led = Constants.opMode.hardwareMap.get(DigitalChannel::class.java, NAME)
    }

    class VerifyAlignment: Command() {
        override val _isDone: Boolean
            get() = Constants.opMode.isStarted
        override val interruptible = true
        override val requirements = listOf(LED)

        override fun execute() {
            led.state = !DistanceSensorAlignment.alignedProperly
        }
    }
}