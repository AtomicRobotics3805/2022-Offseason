@file:Suppress("PropertyName")

package org.firstinspires.ftc.teamcode.commandFramework.driving.localizers

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem

@Suppress("unused", "MemberVisibilityCanBePrivate")
@Config
class DistanceSensorLocalizer(
    private val constants: DistanceSensorLocalizerConstants
) : Localizer {

    override var poseEstimate: Pose2d
        get() = TODO("Not yet implemented")
        set(value) {}
    override val poseVelocity: Pose2d?
        get() = TODO("Not yet implemented")


    private lateinit var forwardDistanceSensor: AnalogInput
    private lateinit var sideDistanceSensor: AnalogInput
    private val forwardDistance: Double
        get() = constants.VOLTAGE_TO_DISTANCE.invoke(forwardDistanceSensor.voltage)
    private val sideDistance: Double
        get() = constants.VOLTAGE_TO_DISTANCE.invoke(sideDistanceSensor.voltage)

    override fun initialize() {
        forwardDistanceSensor = Constants.opMode.hardwareMap.get(AnalogInput::class.java, constants.FORWARD_SENSOR_NAME)
        sideDistanceSensor = Constants.opMode.hardwareMap.get(AnalogInput::class.java, constants.SIDE_SENSOR_NAME)
    }

    override fun update() {

    }
}

interface DistanceSensorLocalizerConstants {
    val FORWARD_SENSOR_NAME: String
    val SIDE_SENSOR_NAME: String
    val FORWARD_SENSOR_REVERSED: Boolean
    val SIDE_SENSOR_REVERSED: Boolean
    val VOLTAGE_TO_DISTANCE: (Double) -> Double
}