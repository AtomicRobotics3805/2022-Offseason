package org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.classification

import android.app.Activity
import android.graphics.Bitmap
import android.graphics.RectF
import android.os.SystemClock
import android.os.Trace
import android.util.Log
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.tensorflow.lite.DataType
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.gpu.GpuDelegate
import org.tensorflow.lite.nnapi.NnApiDelegate
import org.tensorflow.lite.support.common.FileUtil
import org.tensorflow.lite.support.common.TensorOperator
import org.tensorflow.lite.support.common.TensorProcessor
import org.tensorflow.lite.support.image.ImageProcessor
import org.tensorflow.lite.support.image.TensorImage
import org.tensorflow.lite.support.image.ops.ResizeOp
import org.tensorflow.lite.support.image.ops.ResizeWithCropOrPadOp
import org.tensorflow.lite.support.image.ops.Rot90Op
import org.tensorflow.lite.support.label.TensorLabel
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer
import java.io.IOException
import java.nio.MappedByteBuffer
import java.util.ArrayList
import java.util.Comparator
import java.util.PriorityQueue
import java.lang.Math.min
import org.checkerframework.checker.units.qual.min

public abstract class Classifier {
    private val TAG = "Classifier"
    private var telemetry: Telemetry? = null
    private var modelPath: String? = null
    private var labelPath: String? = null

    open fun setModelPath(modelPath: String?) {
        this.modelPath = modelPath
    }

    open fun setLabelPath(labelPath: String?) {
        this.labelPath = labelPath
    }

    /** The model type used for classification.  */
    enum class Model {
        FLOAT_MOBILENET, QUANTIZED_MOBILENET, FLOAT_EFFICIENTNET, QUANTIZED_EFFICIENTNET
    }

    /** The runtime device type used for executing classification.  */
    enum class Device {
        CPU, NNAPI, GPU
    }

    /** Number of results to show in the UI.  */
    private val MAX_RESULTS = 3

    /** The loaded TensorFlow Lite model. */

    /** The loaded TensorFlow Lite model.  */
    /** Image size along the x axis.  */
    private var imageSizeX = 0

    /** Image size along the y axis.  */
    private var imageSizeY = 0

    /** Optional GPU delegate for accleration.  */
    private var gpuDelegate: GpuDelegate? = null

    /** Optional NNAPI delegate for accleration.  */
    private var nnApiDelegate: NnApiDelegate? = null

    /** An instance of the driver class to run model inference with Tensorflow Lite.  */
    protected var tflite: Interpreter? = null

    /** Options for configuring the Interpreter.  */
    private val tfliteOptions = Interpreter.Options()

    /** Labels corresponding to the output of the vision model.  */
    private var labels: List<String>? = null

    /** Input image TensorBuffer.  */
    private var inputImageBuffer: TensorImage? = null

    /** Output probability TensorBuffer.  */
    private var outputProbabilityBuffer: TensorBuffer? = null

    /** Processer to apply post processing of the output probability.  */
    private var probabilityProcessor: TensorProcessor? = null

    /**
     * Creates a classifier with the provided configuration.
     *
     * @param activity The current Activity.
     * @param model The model to use for classification.
     * @param device The device to use for classification.
     * @param numThreads The number of threads to use for classification.
     * @return A classifier with the desired configuration.
     */

    @Throws(Exception::class)
    open fun create(
        activity: Activity?,
        model: Model,
        device: Device?,
        numThreads: Int,
        modelFileName: String?,
        labelFileName: String?,
        t: Telemetry?
    ): Classifier? {
        return if (model === Model.QUANTIZED_MOBILENET) {
            ClassifierQuantizedMobileNet(
                activity,
                device,
                numThreads,
                modelFileName,
                labelFileName,
                t
            )
        } else if (model === Model.FLOAT_MOBILENET) {
            ClassifierFloatMobileNet(activity, device, numThreads, modelFileName, labelFileName, t)
        } else if (model === Model.FLOAT_EFFICIENTNET) {
            ClassifierFloatEfficientNet(
                activity,
                device,
                numThreads,
                modelFileName,
                labelFileName,
                t
            )
        } else if (model === Model.QUANTIZED_EFFICIENTNET) {
            ClassifierQuantizedEfficientNet(
                activity,
                device,
                numThreads,
                modelFileName,
                labelFileName,
                t
            )
        } else {
            throw UnsupportedOperationException()
        }
    }

    /** An immutable result returned by a Classifier describing what was recognized.  */
    class Recognition(
        /**
         * A unique identifier for what has been recognized. Specific to the class, not the instance of
         * the object.
         */
        val id: String?,
        /** Display name for the recognition.  */
        val title: String?,
        /**
         * A sortable score for how good the recognition is relative to others. Higher should be better.
         */
        val confidence: Float?,
        /** Optional location within the source image for the location of the recognized object.  */
        private var location: RectF?
    ) {

        fun getLocation(): RectF {
            return RectF(location)
        }

        fun setLocation(location: RectF?) {
            this.location = location
        }

        override fun toString(): String {
            var resultString = ""
            if (id != null) {
                resultString += "[$id] "
            }
            if (title != null) {
                resultString += "$title "
            }
            if (confidence != null) {
                resultString += String.format("(%.1f%%) ", confidence * 100.0f)
            }
            if (location != null) {
                resultString += location.toString() + " "
            }
            return resultString.trim { it <= ' ' }
        }
    }

    /** Initializes a `Classifier`.  */
    @Throws(Exception::class)
    protected open fun Classifier(
        activity: Activity?,
        device: Device?,
        numThreads: Int,
        modelFileName: String?,
        labelFileName: String?,
        t: Telemetry?
    ) {
        telemetry = t
        setModelPath(modelFileName)
        setLabelPath(labelFileName)
        try {
            val tfliteModel = FileUtil.loadMappedFile(
                activity!!, getModelPath()!!
            )
            when (device) {
                NNAPI -> {
                    nnApiDelegate = NnApiDelegate()
                    tfliteOptions.addDelegate(nnApiDelegate)
                }
                GPU -> {
                    gpuDelegate = GpuDelegate()
                    tfliteOptions.addDelegate(gpuDelegate)
                }
                CPU -> {}
            }
            tfliteOptions.setNumThreads(numThreads)
            tflite = Interpreter(tfliteModel, tfliteOptions)

            // Loads labels out from the label file.
            labels = FileUtil.loadLabels(
                activity,
                getLabelPath()!!
            )
            Log.d(TAG, "Classifier. Labels loaded")

            // Reads type and shape of input and output tensors, respectively.
            val imageTensorIndex = 0
            val imageShape =
                tflite!!.getInputTensor(imageTensorIndex).shape() // {1, height, width, 3}
            imageSizeY = imageShape[1]
            imageSizeX = imageShape[2]
            val imageDataType = tflite!!.getInputTensor(imageTensorIndex).dataType()
            val probabilityTensorIndex = 0
            val probabilityShape =
                tflite!!.getOutputTensor(probabilityTensorIndex).shape() // {1, NUM_CLASSES}
            val probabilityDataType = tflite!!.getOutputTensor(probabilityTensorIndex).dataType()

            // Creates the input tensor.
            inputImageBuffer = TensorImage(imageDataType)

            // Creates the output tensor and its processor.
            outputProbabilityBuffer =
                TensorBuffer.createFixedSize(probabilityShape, probabilityDataType)

            // Creates the post processor for the output probability.
            probabilityProcessor =
                TensorProcessor.Builder().add(getPostprocessNormalizeOp()).build()
            Log.d(TAG, "Created a Tensorflow Lite Image Classifier.")
        } catch (ex: Exception) {
            throw Exception("Unable init classifier", ex)
        }
    }

    /** Runs inference and returns the classification results.  */
    open fun recognizeImage(bitmap: Bitmap, sensorOrientation: Int): List<Recognition?>? {
        var recognitonList: List<Recognition?>? = ArrayList()
        try {
            // Logs this method so that it can be analyzed with systrace.
            Trace.beginSection("recognizeImage")
            Trace.beginSection("loadImage")
            val startTimeForLoadImage = SystemClock.uptimeMillis()
            inputImageBuffer = loadImage(bitmap, sensorOrientation)
            val endTimeForLoadImage = SystemClock.uptimeMillis()
            Trace.endSection()
            Log.v(TAG, "Time to load the image: " + (endTimeForLoadImage - startTimeForLoadImage))

            // Runs the inference call.
            Trace.beginSection("runInference")
            val startTimeForReference = SystemClock.uptimeMillis()
            tflite!!.run(inputImageBuffer.getBuffer(), outputProbabilityBuffer!!.buffer.rewind())
            val endTimeForReference = SystemClock.uptimeMillis()
            Trace.endSection()
            Log.v(
                TAG,
                "Time to run model inference: " + (endTimeForReference - startTimeForReference)
            )

            // Gets the map of label and probability.
            val labeledProbability =
                TensorLabel(labels!!, probabilityProcessor!!.process(outputProbabilityBuffer))
                    .mapWithFloatValue
            Trace.endSection()

            // Gets top-k results.
            recognitonList = getTopKProbability(labeledProbability)
        } catch (ex: Exception) {
            Log.e(TAG, String.format("Error in recognizeImage. %s", ex.toString()))
        }
        return recognitonList
    }

    /** Closes the interpreter and model to release resources.  */
    open fun close() {
        if (tflite != null) {
            tflite!!.close()
            tflite = null
        }
        if (gpuDelegate != null) {
            gpuDelegate.close()
            gpuDelegate = null
        }
        if (nnApiDelegate != null) {
            nnApiDelegate.close()
            nnApiDelegate = null
        }
    }

    /** Get the image size along the x axis.  */
    open fun getImageSizeX(): Int {
        return imageSizeX
    }

    /** Get the image size along the y axis.  */
    open fun getImageSizeY(): Int {
        return imageSizeY
    }

    /** Loads input image, and applies preprocessing.  */
    private open fun loadImage(bitmap: Bitmap, sensorOrientation: Int): TensorImage {
        // Loads bitmap into a TensorImage.
        inputImageBuffer!!.load(bitmap)

        // Creates processor for the TensorImage.
        val cropSize = min(bitmap.width, bitmap.height)
        val numRotation = sensorOrientation / 90
        // TODO(b/143564309): Fuse ops inside ImageProcessor.
        val imageProcessor = ImageProcessor.Builder()
            .add(ResizeWithCropOrPadOp(cropSize, cropSize))
            .add(ResizeOp(imageSizeX, imageSizeY, ResizeOp.ResizeMethod.NEAREST_NEIGHBOR))
            .add(Rot90Op(numRotation))
            .add(getPreprocessNormalizeOp())
            .build()
        return imageProcessor.process(inputImageBuffer)
    }

    /** Gets the top-k results.  */
    private open fun getTopKProbability(labelProb: Map<String, Float>): List<Recognition?>? {
        // Find the best classifications.
        val pq = PriorityQueue<Recognition>(
            MAX_RESULTS
        ) { lhs, rhs -> // Intentionally reversed to put high confidence at the head of the queue.
            java.lang.Float.compare(rhs.getConfidence(), lhs.getConfidence())
        }
        for ((key, value) in labelProb) {
            pq.add(Recognition("" + key, key, value, null))
        }
        val recognitions = ArrayList<Recognition?>()
        val recognitionsSize = min(pq.size, MAX_RESULTS)
        for (i in 0 until recognitionsSize) {
            recognitions.add(pq.poll())
        }
        return recognitions
    }


    protected open fun getModelPath(): String? {
        return modelPath
    }


    protected open fun getLabelPath(): String? {
        return labelPath
    }

    /** Gets the TensorOperator to nomalize the input image in preprocessing.  */
    protected abstract fun getPreprocessNormalizeOp(): TensorOperator?

    /**
     * Gets the TensorOperator to dequantize the output probability in post processing.
     *
     *
     * For quantized model, we need de-quantize the prediction with NormalizeOp (as they are all
     * essentially linear transformation). For float model, de-quantize is not required. But to
     * uniform the API, de-quantize is added to float model too. Mean and std are set to 0.0f and
     * 1.0f, respectively.
     */
    protected abstract fun getPostprocessNormalizeOp(): TensorOperator?
}

