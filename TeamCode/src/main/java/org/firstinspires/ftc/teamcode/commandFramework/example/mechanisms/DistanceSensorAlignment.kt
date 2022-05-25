package org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem

@Suppress("unused", "MemberVisibilityCanBePrivate")
@Config
object DistanceSensorAlignment : Subsystem {

    @JvmField
    var NAME = "distanceSensor"
    @JvmField
    var TARGET_DISTANCE = Pair(10.0, 50.0)

    val alignedProperly: Boolean
        get() = voltageToDistance(distanceSensor.voltage) in
                TARGET_DISTANCE.first..TARGET_DISTANCE.second

    private lateinit var distanceSensor: AnalogInput

    override fun initialize() {
        distanceSensor = Constants.opMode.hardwareMap.get(AnalogInput::class.java, NAME)
    }

    private fun voltageToDistance(voltage: Double) = voltage * 30.0
}