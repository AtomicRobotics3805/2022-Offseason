package org.firstinspires.ftc.teamcode.main.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.commandFramework.utilCommands.CustomCommand

@Suppress("unused")
@Config
object ContainerSensor : Subsystem {
    @JvmField
    var NAME = "sensor_color"

    enum class ContainerState {
        EMPTY,
        FULL,
        UNKNOWN
    }

    val containerState: ContainerState
        get() = if((containerSensor as DistanceSensor).getDistance(DistanceUnit.CM) > 3.35) ContainerState.EMPTY else ContainerState.FULL

    private lateinit var containerSensor: NormalizedColorSensor

    override fun initialize() {
        containerSensor = Constants.opMode.hardwareMap.get(NormalizedColorSensor::class.java, NAME)
    }
}