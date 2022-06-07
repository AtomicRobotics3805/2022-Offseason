package org.firstinspires.ftc.teamcode.main.testing.tuning

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.TelemetryController
import org.firstinspires.ftc.teamcode.commandFramework.hardware.DistanceSensor
import org.firstinspires.ftc.teamcode.commandFramework.hardware.MBUltrasonic

/**
 * This simple tuning class uses the DistanceSensor hardware class to translate voltage into
 * distance. It displays the distance, voltage, minDistance, and maxDistance through telemetry.
 */
@TeleOp(name = "Distance Sensor Tuner")
class DistanceSensorTuner: LinearOpMode() {

    private lateinit var distanceSensor: DistanceSensor

    /**
     * Runs the tuning program
     */
    override fun runOpMode() {
        Constants.opMode = this
        TelemetryController.initialize()
        distanceSensor = MBUltrasonic(DISTANCE_SENSOR_NAME)
        waitForStart()
        while (opModeIsActive()) {
            TelemetryController.telemetry.addData("Min Distance", distanceSensor.minDistance)
            TelemetryController.telemetry.addData("Max Distance", distanceSensor.maxDistance)
            TelemetryController.telemetry.addData("Voltage", distanceSensor.voltage)
            TelemetryController.telemetry.addData("Distance", distanceSensor.distance)
            TelemetryController.periodic()
        }
    }

    companion object {
        var DISTANCE_SENSOR_NAME = "distanceSensor"
    }
}

