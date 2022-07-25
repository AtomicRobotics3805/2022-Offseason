package org.firstinspires.ftc.teamcode.commandFramework.detection.openCV

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.*
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.utilCommands.CustomCommand
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.*
import org.openftc.easyopencv.OpenCvCameraFactory

class WebcamDetection {
    var cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
        "cameraMonitorViewId",
        "id",
        hardwareMap.appContext.packageName
    )

    private val VERTICAL_RESOLUTION = 1080
    private val HORIZONTAL_RESOLUTION = 1920

    val start: Command
        get() = startStream()
    val pause: Command
        get() = pauseStream()
    val resume: Command
        get() = resumeStream()

    private lateinit var camera: OpenCvWebcam

    fun initialize(){
        camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(
                WebcamName::class.java, "Webcam 1"
            ), cameraMonitorViewId
        )
        val examplePipeline = ExamplePipeline(telemetry)
        camera.setPipeline(examplePipeline)
    }

    fun startStream() = CustomCommand(_start ={
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(VERTICAL_RESOLUTION, HORIZONTAL_RESOLUTION, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
                // This will be called if the camera could not be opened
            }
        })
    })

    fun pauseStream() = CustomCommand(_start ={
        camera.pauseViewport()
    })
    fun resumeStream() = CustomCommand(_start = {
        camera.resumeViewport()
    })

}