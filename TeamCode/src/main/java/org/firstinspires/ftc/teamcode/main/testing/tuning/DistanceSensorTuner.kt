package org.firstinspires.ftc.teamcode.main.testing.tuning

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.AnalogInput

class DistanceSensorTuner: LinearOpMode() {

    lateinit var distanceSensor: AnalogInput

    override fun runOpMode() {
        TODO("Not yet implemented")
    }

    companion object {
        var DISTANCE_SENSOR_NAME = "distanceSensor"
    }
}

