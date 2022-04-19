package org.firstinspires.ftc.teamcode.commandFramework.detection.openCV

import org.openftc.easyopencv.OpenCvPipeline
import org.opencv.core.Mat

object Pipeline : OpenCvPipeline() {
    override fun processFrame(input: Mat?): Mat? {
        return input
    }
}