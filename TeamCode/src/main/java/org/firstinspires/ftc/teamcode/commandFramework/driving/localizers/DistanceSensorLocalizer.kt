@file:Suppress("PropertyName")

package org.firstinspires.ftc.teamcode.commandFramework.driving.localizers

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.Constants.drive
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.toRadians
import kotlin.math.cos

/**
 * Note: this localizer only works when you know that the forward distance sensor will be hitting
 * the forward wall and the side sensor will be hitting the side wall. So basically, it only works
 * if the robot isn't rotating enough for one of the distance sensors to hit a different wall. This
 * is relevant in certain games like Freight Frenzy, but not in others.
 */
@Suppress("unused", "MemberVisibilityCanBePrivate")
@Config
class DistanceSensorLocalizer(
    private val constants: DistanceSensorLocalizerConstants
) : Localizer {

    override var poseEstimate: Pose2d
        get() {
            val x = sideDistance * cos(360.0.toRadians - drive.rawExternalHeading) - 72.0
            val y = if (Constants.color == Constants.Color.BLUE)
                forwardDistance * cos(360.0.toRadians - drive.rawExternalHeading) - 72.0
            else -forwardDistance * cos(360.0.toRadians - drive.rawExternalHeading) + 144.0
            return Pose2d(x, y, drive.rawExternalHeading)
        }
        set(value) { }
    override val poseVelocity: Pose2d? = null

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
    val VOLTAGE_TO_DISTANCE: (Double) -> Double
}