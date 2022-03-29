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

    private val VERTICAL_RESOLUTION = 0
    private val HORIZONTAL_RESOLUTION = 0

    val start: Command
        get() = startStream()
    val pause: Command
        get() = streamControl("pause")
    val resume: Command
        get() = streamControl("resume")

    private lateinit var camera: OpenCvWebcam

    fun initialize(){
        camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(
                WebcamName::class.java, "Webcam 1"
            ), cameraMonitorViewId
        )
        camera.setPipeline(Pipeline)
    }

    fun startStream() = CustomCommand(_start ={
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(VERTICAL_RESOLUTION,HORIZONTAL_RESOLUTION, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
                // This will be called if the camera could not be opened
            }
        })
    })

    fun streamControl(request:String) = CustomCommand(_start = {
        if (request.equals("pause")){
            camera.pauseViewport()
        }
        else if (request.equals("resume")){
            camera.resumeViewport()
        }
    })

}