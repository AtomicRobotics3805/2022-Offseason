package org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.classification

import android.app.Activity;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.tensorflow.lite.support.common.TensorOperator;
import org.tensorflow.lite.support.common.ops.NormalizeOp;
import java.lang.Exception

class ClassifierQuantizedMobileNet(activity: Activity?,
                                   device: Device?,
                                   numThreads: Int,
                                   modelFileName: String?,
                                   labelFileName: String?,
                                   t: Telemetry?): Classifier(activity, device, numThreads, modelFileName, labelFileName, t) {
    private val IMAGE_MEAN = 0.0f

    private val IMAGE_STD = 1.0f

    /** Quantized MobileNet requires additional dequantization to the output probability.  */
    private val PROBABILITY_MEAN = 0.0f

    private val PROBABILITY_STD = 255.0f

    /**
     * Initializes a `ClassifierQuantizedMobileNet`.
     *
     * @param activity
     */
    @Throws(Exception::class)

    override fun getPreprocessNormalizeOp(): TensorOperator? {
        return NormalizeOp(IMAGE_MEAN, IMAGE_STD)
    }

    override fun getPostprocessNormalizeOp(): TensorOperator? {
        return NormalizeOp(PROBABILITY_MEAN, PROBABILITY_STD)
    }
}