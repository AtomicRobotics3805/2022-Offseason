package org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow

import android.app.Activity;
import android.app.Fragment;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Typeface;
import android.hardware.Camera;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.media.Image
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.os.Trace
import android.util.Log
import android.util.Size
import android.util.TypedValue
import android.view.Surface
import android.view.View
import android.widget.FrameLayout

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.R
import org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.classification.Classifier.Device;
import org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.classification.Classifier.Model;
import org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.utils.BorderedText;
import org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.utils.ImageUtils;
import org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.views.CameraConnectionFragment;
import org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.views.LegacyCameraConnectionFragment;

import java.lang.Exception


class Detector: ImageReader.OnImageAvailableListener, Camera.PreviewCallback{
    private val TAG = "Detector"
    private var rgbFrameBitmap: Bitmap? = null
    var cameraId: String? = null
    private var sensorOrientation: Int? = null
    private var borderedText: BorderedText? = null
    private var rgbBytes: IntArray? = null
    private val yuvBytes = arrayOfNulls<ByteArray>(3)
    private var imageConverter: Runnable? = null
    private var postInferenceCallback: Runnable? = null
    private var handler: Handler? = null
    private var handlerThread: HandlerThread? = null
    protected var previewWidth = 0
    protected var previewHeight = 0
    private var yRowStride = 0
    var telemetry: Telemetry? = null
    var useCamera2API = false
    private var isProcessingFrame = false
    private var appContext: Context? = null
    private var classifier: Classifier? = null
    private var model = Model.QUANTIZED_EFFICIENTNET
    private var device = Device.CPU
    private var modelPath: String? = null
    private var labelPath: String? = null
    private var numThreads = 1
    private val TEXT_SIZE_DIP = 10f
    private val DESIRED_PREVIEW_SIZE = Size(640, 480)
    private var lastResults: kotlin.collections.List<Classifier.Recognition?>? = null
    private var tfodMonitorViewId = -1
    private var fragment: Fragment? = null

    /** Input image size of the model along x axis.  */
    private var imageSizeX = 0

    /** Input image size of the model along y axis.  */
    private var imageSizeY = 0


    @Throws(Exception::class)
    fun Detector(modelType: Model, modelPath: String, ctx: Context?, t: Telemetry?) {
        var modelFileName = modelPath.substring(0, modelPath.lastIndexOf('.'))
        val labelFileName = String.format("%s_labels.txt", modelFileName)
        modelFileName = String.format("%s.tflite", modelFileName)
        init(modelType, modelFileName, labelFileName, ctx, t)
    }

    @Throws(Exception::class)
    fun Detector(
        modelType: Model,
        modelPath: String?,
        labelPath: String?,
        ctx: Context?,
        t: Telemetry?
    ) {
        init(modelType, modelPath, labelPath, ctx, t)
    }

    @Throws(Exception::class)
    protected fun init(
        modelType: Model,
        modelPath: String?,
        labelPath: String?,
        ctx: Context?,
        t: Telemetry?
    ) {
        Log.d(TAG, "Classifier. Starting Init method")
        appContext = ctx
        telemetry = t
        tfodMonitorViewId = appContext!!.resources.getIdentifier(
            "tfodMonitorViewId", "id", appContext!!.packageName
        )
        try {
            (appContext as Activity?)!!.runOnUiThread { //make visible
                val fm =
                    (appContext as Activity?)!!.findViewById<View>(tfodMonitorViewId) as FrameLayout
                if (fm != null) {
                    fm.visibility = View.VISIBLE
                }
            }
            setModelType(modelType)
            setModelPath(modelPath)
            setLabelPath(labelPath)
            Log.d(TAG, "Classifier. Init method complete")
        } catch (e: Exception) {
            Log.e(TAG, "Init method error", e)
            throw Exception("Make frame visible", e)
        }
    }

    @Synchronized
    protected fun startProcessing() {
        handlerThread = HandlerThread("inference")
        handlerThread!!.start()
        handler = Handler(handlerThread!!.looper)
    }

    @Synchronized
    fun stopProcessing() {
        if (handlerThread == null) {
            return
        }
        handlerThread!!.quitSafely()
        try {
            handlerThread!!.join()
            handlerThread = null
            handler = null
            if (fragment != null) {
                (appContext as Activity?)!!.fragmentManager.beginTransaction().remove(fragment)
                    .commit()
            }
        } catch (e: InterruptedException) {
            Log.e(TAG, "Unable to stop processing", e)
        }
    }


    @Throws(Exception::class)
    fun activate() {
        startProcessing()
        val manager = appContext!!.getSystemService(Context.CAMERA_SERVICE) as CameraManager
        try {
            for (cameraId in manager.cameraIdList) {
                val characteristics = manager.getCameraCharacteristics(
                    cameraId!!
                )
                Log.d(TAG, String.format("Activation. Cam ID: %s", cameraId))
                val facing = characteristics.get(CameraCharacteristics.LENS_FACING)
                Log.d(TAG, String.format("Activation. Facing: %d", facing))
                val orientation = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION)
                Log.d(TAG, String.format("Activation. Orientation: %d", orientation))
                if (facing != null && facing == CameraCharacteristics.LENS_FACING_BACK) {
                    val map =
                        characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
                            ?: continue

                    // Fallback to camera1 API for internal cameras that don't have full support.
                    // This should help with legacy situations where using the camera2 API causes
                    // distorted or otherwise broken previews.
                    useCamera2API = (facing == CameraCharacteristics.LENS_FACING_EXTERNAL
                            || isHardwareLevelSupported(
                        characteristics, CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_FULL
                    ))
                    Log.d(TAG, String.format("Activation. Camera API lv2?: %b", useCamera2API))
                    this.cameraId = cameraId
                    break
                }
            }
            Log.d(TAG, String.format("Activation. Selected Camera: %s", cameraId))
            setFragment()
            Log.d(TAG, "Activation. Complete")
        } catch (e: CameraAccessException) {
            Log.e(TAG, "Not allowed to access camera", e)
            throw Exception("Not allowed to access camera", e)
        } catch (e: Exception) {
            Log.e(TAG, "Problems with activation", e)
            throw Exception("Problems with activation", e)
        }
    }

    protected fun setFragment() {
        if (useCamera2API) {
            val camera2Fragment: CameraConnectionFragment = CameraConnectionFragment.newInstance(
                object : CameraConnectionFragment.ConnectionCallback {
                    override fun onPreviewSizeChosen(size: Size?, rotation: Int) {
                        previewHeight = size!!.height
                        previewWidth = size.width
                        this@Detector.onPreviewSizeChosen(size, rotation)
                    }
                },
                this,
                getLayoutId(),
                getDesiredPreviewFrameSize(), telemetry
            )
            camera2Fragment.setCamera(cameraId)
            fragment = camera2Fragment
        } else {
            fragment = LegacyCameraConnectionFragment(
                this,
                getLayoutId(),
                getDesiredPreviewFrameSize(),
                telemetry
            )
        }
        (appContext as Activity?)!!.fragmentManager.beginTransaction()
            .replace(tfodMonitorViewId, fragment).commit()
        Log.d(TAG, "SetFragment. Complete")
    }

    protected fun getLayoutId(): Int {
        return R.layout.tfe_ic_camera_connection_fragment
    }

    protected fun getDesiredPreviewFrameSize(): Size? {
        return DESIRED_PREVIEW_SIZE
    }

    private fun isHardwareLevelSupported(
        characteristics: CameraCharacteristics, requiredLevel: Int
    ): Boolean {
        val deviceLevel = characteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL)!!
        return if (deviceLevel == CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY) {
            requiredLevel == deviceLevel
        } else requiredLevel <= deviceLevel
        // deviceLevel is not LEGACY, can use numerical sort
    }


    protected fun getRgbBytes(): IntArray? {
        imageConverter!!.run()
        return rgbBytes
    }

    protected fun fillBytes(planes: Array<Image.Plane>, yuvBytes: Array<ByteArray?>) {
        // Because of the variable row stride it's not possible to know in
        // advance the actual necessary dimensions of the yuv planes.
        for (i in planes.indices) {
            val buffer = planes[i].buffer
            if (yuvBytes[i] == null) {
                yuvBytes[i] = ByteArray(buffer.capacity())
            }
            buffer[yuvBytes[i]]
        }
    }


    protected fun readyForNextImage() {
        if (postInferenceCallback != null) {
            postInferenceCallback!!.run()
        }
    }

    protected fun getScreenOrientation(): Int {
        return when ((appContext as Activity?)!!.windowManager.defaultDisplay.rotation) {
            Surface.ROTATION_270 -> 270
            Surface.ROTATION_180 -> 180
            Surface.ROTATION_90 -> 90
            else -> 0
        }
    }

    @Synchronized
    protected fun runInBackground(r: Runnable?) {
        if (handler != null) {
            handler!!.post(r)
        }
    }

    fun onPreviewFrame(bytes: ByteArray?, camera: Camera) {
        if (isProcessingFrame) {
            Log.d(TAG, "onPreviewFrame. Dropping frame")
            return
        }
        try {
            // Initialize the storage bitmaps once when the resolution is known.
            if (rgbBytes == null) {
                val previewSize = camera.parameters.previewSize
                previewHeight = previewSize.height
                previewWidth = previewSize.width
                rgbBytes = IntArray(previewWidth * previewHeight)
                onPreviewSizeChosen(Size(previewSize.width, previewSize.height), 90)
            }
        } catch (e: Exception) {
            Log.e(TAG, "onPreviewFrame. Error", e)
            return
        }
        if (classifier == null) {
            Log.e(TAG, "No classifier on preview!")
            return
        }
        isProcessingFrame = true
        yuvBytes[0] = bytes
        yRowStride = previewWidth
        imageConverter = Runnable {
            ImageUtils.convertYUV420SPToARGB8888(
                bytes,
                previewWidth,
                previewHeight,
                rgbBytes
            )
        }
        postInferenceCallback = Runnable {
            camera.addCallbackBuffer(bytes)
            isProcessingFrame = false
        }
        processImage()
    }

    fun onPreviewSizeChosen(size: Size?, rotation: Int) {
        val textSizePx = TypedValue.applyDimension(
            TypedValue.COMPLEX_UNIT_DIP, TEXT_SIZE_DIP, appContext!!.resources.displayMetrics
        )
        borderedText = BorderedText(textSizePx)
        borderedText!!.setTypeface(Typeface.MONOSPACE)
        recreateClassifier(getModelType(), getDevice(), getNumThreads())
        if (classifier == null) {
            Log.e(TAG, "onPreviewSizeChosen. No classifier on preview!")
            return
        }
        previewWidth = size!!.width
        previewHeight = size.height
        sensorOrientation = rotation - getScreenOrientation()
        Log.d(
            TAG,
            String.format(
                "onPreviewSizeChosen. Camera orientation relative to screen canvas: %d",
                sensorOrientation
            )
        )
        Log.d(
            TAG,
            String.format(
                "onPreviewSizeChosen. Initializing at size %dx%ds:",
                previewWidth,
                previewHeight
            )
        )
        rgbFrameBitmap = Bitmap.createBitmap(previewWidth, previewHeight, Bitmap.Config.ARGB_8888)
    }

    protected fun processImage() {
        rgbFrameBitmap!!.setPixels(
            getRgbBytes(),
            0,
            previewWidth,
            0,
            0,
            previewWidth,
            previewHeight
        )
        //        final int cropSize = Math.min(previewWidth, previewHeight);
        runInBackground {
            if (classifier != null) {
                lastResults = classifier!!.recognizeImage(rgbFrameBitmap!!, sensorOrientation!!)
            } else {
                Log.e(TAG, "processImage. No Classifier to process image")
            }
            readyForNextImage()
        }
    }

    fun onImageAvailable(imageReader: ImageReader) {
        if (previewWidth == 0 || previewHeight == 0) {
            return
        }
        if (rgbBytes == null) {
            rgbBytes = IntArray(previewWidth * previewHeight)
        }
        try {
            val image = imageReader.acquireLatestImage() ?: return
            if (isProcessingFrame) {
                image.close()
                return
            }
            isProcessingFrame = true
            Trace.beginSection("imageAvailable")
            val planes = image.planes
            fillBytes(planes, yuvBytes)
            yRowStride = planes[0].rowStride
            val uvRowStride = planes[1].rowStride
            val uvPixelStride = planes[1].pixelStride
            imageConverter = Runnable {
                ImageUtils.convertYUV420ToARGB8888(
                    yuvBytes[0],
                    yuvBytes[1],
                    yuvBytes[2],
                    previewWidth,
                    previewHeight,
                    yRowStride,
                    uvRowStride,
                    uvPixelStride,
                    rgbBytes
                )
            }
            postInferenceCallback = Runnable {
                image.close()
                isProcessingFrame = false
            }
            processImage()
        } catch (e: Exception) {
            Log.e(TAG, "onImageAvailable error", e)
            Trace.endSection()
            return
        }
        Trace.endSection()
    }

    protected fun onInferenceConfigurationChanged() {
        if (rgbFrameBitmap == null) {
            // Defer creation until we're getting camera frames.
            return
        }
        val device = getDevice()
        val model = getModelType()
        val numThreads = getNumThreads()
        runInBackground { recreateClassifier(model, device, numThreads) }
    }

    private fun recreateClassifier(model: Model, device: Device, numThreads: Int) {
        if (classifier != null) {
            Log.d(TAG, "recreateClassifier. Closing classifier")
            classifier!!.close()
            classifier = null
        }
        if (device === Device.GPU
            && (model === Model.QUANTIZED_MOBILENET || model === Model.QUANTIZED_EFFICIENTNET)
        ) {
            Log.e(
                TAG,
                "recreateClassifier. Not creating classifier: GPU doesn't support quantized models."
            )
            return
        }
        try {
            classifier = Classifier.create(
                appContext as Activity?, model, device, numThreads,
                getModelPath(), getLabelPath(), telemetry
            )
            Log.d(
                TAG,
                java.lang.String.format(
                    "Created classifier (model=%s, device=%s, numThreads=%d)",
                    model,
                    device,
                    numThreads
                )
            )
        } catch (e: Exception) {
            Log.e(TAG, "Failed to create classifier", e)
        }
        if (classifier != null) {
            // Updates the input image size.
            imageSizeX = classifier.getImageSizeX()
            imageSizeY = classifier.getImageSizeY()
            Log.d(TAG, "Classifier done")
        } else {
            Log.e(TAG, "Failed to create classifier")
        }
    }

    protected fun getModelType(): Model {
        return model
    }

    fun setModelType(model: Model) {
        if (this.model !== model) {
            this.model = model
            Log.d(TAG, java.lang.String.format("Updated model %s", model))
            //            onInferenceConfigurationChanged();
        }
    }

    protected fun getDevice(): Device {
        return device
    }


    private fun setDevice(device: Device) {
        if (this.device !== device) {
            this.device = device
            Log.d(TAG, java.lang.String.format("Updated device %s", device))
            val threadsEnabled = device === Device.CPU
            onInferenceConfigurationChanged()
        }
    }

    protected fun getNumThreads(): Int {
        return numThreads
    }

    private fun setNumThreads(numThreads: Int) {
        if (this.numThreads != numThreads) {
            this.numThreads = numThreads
            onInferenceConfigurationChanged()
        }
    }

    fun getLastResults(): kotlin.collections.List<Classifier.Recognition?>? {
        return lastResults
    }

    fun getModelPath(): String? {
        return modelPath
    }

    fun setModelPath(modelPath: String?) {
        this.modelPath = modelPath
    }

    fun getLabelPath(): String? {
        return labelPath
    }

    fun setLabelPath(labelPath: String?) {
        this.labelPath = labelPath
    }
}