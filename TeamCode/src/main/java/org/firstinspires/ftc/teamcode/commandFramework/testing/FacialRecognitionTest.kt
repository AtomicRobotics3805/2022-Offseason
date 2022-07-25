package org.firstinspires.ftc.teamcode.commandFramework.testing

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.detectors.ExampleDetector

@TeleOp(name = "Facial Recognition", group = "Testing")
public class FacialRecognitionTest : LinearOpMode(){

    var exampleDetector: ExampleDetector? = null

    override fun runOpMode(){
        exampleDetector = ExampleDetector(this.hardwareMap, this, telemetry)
        var detectThread: Thread = Thread(exampleDetector)
        detectThread.start()
        telemetry.update()

        waitForStart()

        var face: String? = null

        while (opModeIsActive()){
            face = exampleDetector!!.result
            telemetry.addData("Person =", face)
        }
        exampleDetector!!.stopDetection()

    }

}