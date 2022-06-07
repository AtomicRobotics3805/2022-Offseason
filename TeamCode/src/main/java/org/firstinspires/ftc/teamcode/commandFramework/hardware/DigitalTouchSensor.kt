package org.firstinspires.ftc.teamcode.commandFramework.hardware

import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.commandFramework.Constants

class DigitalTouchSensor(name: String) {

    val pressed: Boolean
        get() = input.state
    private val input: DigitalChannel =
        Constants.opMode.hardwareMap.get(DigitalChannel::class.java, name)
}