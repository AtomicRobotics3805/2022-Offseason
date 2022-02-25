package org.firstinspires.ftc.teamcode.commandFramework.detection

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.openftc.easyopencv.OpenCvInternalCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCamera
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms.Intake
import org.firstinspires.ftc.teamcode.commandFramework.utilCommands.CustomCommand
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraRotation
import org.opencv.core.Mat

import org.openftc.easyopencv.OpenCvPipeline

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

    private lateinit var camera: OpenCvCamera

    fun initialize(){
        // With live preview
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId)

        // Without live preview
        //camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK)

        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
            }

            override fun onError(errorCode: Int) {
                /*
       * This will be called if the camera could not be opened
       */
            }
        })

        camera.setPipeline(Pipeline)
    }

    fun startStream() = CustomCommand(_start ={
        camera.startStreaming(VERTICAL_RESOLUTION,HORIZONTAL_RESOLUTION, OpenCvCameraRotation.UPRIGHT)
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