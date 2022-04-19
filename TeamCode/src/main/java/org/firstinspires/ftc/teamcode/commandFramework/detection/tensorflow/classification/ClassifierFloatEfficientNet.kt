package org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.classification
import android.app.Activity;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.tensorflow.lite.support.common.TensorOperator;
import org.tensorflow.lite.support.common.ops.NormalizeOp;

import java.lang.Exception

class ClassifierFloatEfficientNet: Classifier(){
    private val IMAGE_MEAN = 127.0f
    private val IMAGE_STD = 128.0f

    /**
     * Float model does not need dequantization in the post-processing. Setting mean and std as 0.0f
     * and 1.0f, repectively, to bypass the normalization.
     */
    private val PROBABILITY_MEAN = 0.0f

    private val PROBABILITY_STD = 1.0f

    /**
     * Initializes a `ClassifierFloatMobileNet`.
     *
     * @param activity
     */
    @Throws(Exception::class)

    constructor (activity: Activity?,
                 device: Device?,
                 numThreads: Int,
                 modelFileName: String?,
                 labelFileName: String?,
                 t: Telemetry?): this(activity, device, numThreads, modelFileName, labelFileName, t)

    override fun getPreprocessNormalizeOp(): TensorOperator? {
        return NormalizeOp(IMAGE_MEAN, IMAGE_STD)
    }

    override fun getPostprocessNormalizeOp(): TensorOperator? {
        return NormalizeOp(PROBABILITY_MEAN, PROBABILITY_STD)
    }

}