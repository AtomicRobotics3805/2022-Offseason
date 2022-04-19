package org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.views

import android.annotation.SuppressLint;
import android.app.Fragment;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.util.Size;
import android.util.SparseIntArray;
import android.view.LayoutInflater;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.view.ViewGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.utils.ImageUtils;

import java.io.IOException;
import java.util.List;
import android.R
import android.hardware.Camera.*
import android.view.TextureView.SurfaceTextureListener


class LegacyCameraConnectionFragment: Fragment() {
    private val ORIENTATIONS = SparseIntArray()
    private val TAG = "Fragment"

    static
    {
        ORIENTATIONS.append(Surface.ROTATION_0, 90)
        ORIENTATIONS.append(Surface.ROTATION_90, 0)
        ORIENTATIONS.append(Surface.ROTATION_180, 270)
        ORIENTATIONS.append(Surface.ROTATION_270, 180)
    }

    private var camera: Camera? = null
    private var imageListener: PreviewCallback? = null
    private var desiredSize: Size? = null
    private var telemetry: Telemetry? = null

    /**
     * The layout identifier to inflate for this Fragment.
     */
    private var layout = 0

    /**
     * An [AutoFitTextureView] for camera preview.
     */
    private var textureView: AutoFitTextureView? = null

    /**
     * [TextureView.SurfaceTextureListener] handles several lifecycle events on a [ ].
     */
    private val surfaceTextureListener: SurfaceTextureListener = object : SurfaceTextureListener {
        override fun onSurfaceTextureAvailable(
            texture: SurfaceTexture, width: Int, height: Int
        ) {
            Log.d("Listener", "Starting texture listener")
            //This variable is required for the fix for the Android media center issue when the camera fails to initialize
            val listener: SurfaceTextureListener = this
            val index = getCameraId()
            Log.d("cameraIndex", index.toString())
            camera = open(index)
            camera.setErrorCallback(ErrorCallback { i, camera ->
                Log.e(
                    "Listener",
                    String.format("Camera error %d. Width: %d, Height: %d", i, width, height)
                )
                //Fix for the Android media center issue when the camera fails to initialize
                if (i == 100) {
                    Log.d("Listener", "Camera error is 100")
                    stopCamera()
                    Log.d("Listener", "Stopped and released the camera")
                    listener.onSurfaceTextureAvailable(texture, width, height)
                }
            })
            try {
//                        camera.stopPreview();
//                        camera.release();
                val parameters = camera.getParameters()
                val focusModes = parameters.supportedFocusModes
                if (focusModes != null
                    && focusModes.contains(Parameters.FOCUS_MODE_CONTINUOUS_PICTURE)
                ) {
                    parameters.focusMode = Parameters.FOCUS_MODE_CONTINUOUS_PICTURE
                }
                val cameraSizes = parameters.supportedPreviewSizes
                val sizes = arrayOfNulls<Size>(cameraSizes.size)
                var i = 0
                for (size in cameraSizes) {
                    sizes[i++] = Size(size.width, size.height)
                }
                val previewSize: Size = CameraConnectionFragment.chooseOptimalSize(
                    sizes, desiredSize!!.width, desiredSize!!.height, telemetry
                )
                parameters.setPreviewSize(previewSize.width, previewSize.height)
                camera.setDisplayOrientation(90)
                camera.setParameters(parameters)
                camera.setPreviewTexture(texture)
            } catch (exception: IOException) {
                Log.e("Listener", "Error when starting texture listener", exception)
                camera.release()
            }
            camera.setPreviewCallbackWithBuffer(imageListener)
            val s = camera.getParameters().previewSize
            camera.addCallbackBuffer(ByteArray(ImageUtils.getYUVByteSize(s.height, s.width)))
            textureView!!.setAspectRatio(s.height, s.width)
            camera.startPreview()
            Log.d("Listener", "Camera preview started by texture listener")
        }

        override fun onSurfaceTextureSizeChanged(
            texture: SurfaceTexture, width: Int, height: Int
        ) {
        }

        override fun onSurfaceTextureDestroyed(texture: SurfaceTexture): Boolean {
            return true
        }

        override fun onSurfaceTextureUpdated(texture: SurfaceTexture) {}
    }

    /**
     * An additional thread for running tasks that shouldn't block the UI.
     */
    private var backgroundThread: HandlerThread? = null

    @SuppressLint("ValidFragment")
    fun LegacyCameraConnectionFragment(
        imageListener: PreviewCallback?, layout: Int, desiredSize: Size?, t: Telemetry?
    ) {
        this.imageListener = imageListener
        this.layout = layout
        this.desiredSize = desiredSize
        telemetry = t
    }

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?
    ): View? {
        return inflater.inflate(layout, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        textureView = view.findViewById<View>(R.id.texture) as AutoFitTextureView
    }

    override fun onActivityCreated(savedInstanceState: Bundle?) {
        super.onActivityCreated(savedInstanceState)
    }

    override fun onResume() {
        super.onResume()
        startBackgroundThread()
        // When the screen is turned off and turned back on, the SurfaceTexture is already
        // available, and "onSurfaceTextureAvailable" will not be called. In that case, we can open
        // a camera and start preview from here (otherwise, we wait until the surface is ready in
        // the SurfaceTextureListener).
        Log.d(TAG, "Starting texture preview")
        if (textureView!!.isAvailable) {
            if (camera != null) {
                camera!!.startPreview()
                Log.d(TAG, "Started preview")
            } else {
                Log.e(TAG, "Failed to start preview. Camera unavailable")
            }
        } else {
            Log.d(TAG, "Texture not available yet. Starting the listener...")
            textureView!!.surfaceTextureListener = surfaceTextureListener
        }
    }

    override fun onPause() {
        stopCamera()
        stopBackgroundThread()
        super.onPause()
    }

    /**
     * Starts a background thread and its [Handler].
     */
    private fun startBackgroundThread() {
        backgroundThread = HandlerThread("CameraBackground")
        backgroundThread!!.start()
    }

    /**
     * Stops the background thread and its [Handler].
     */
    private fun stopBackgroundThread() {
        backgroundThread!!.quitSafely()
        try {
            backgroundThread!!.join()
            backgroundThread = null
        } catch (e: InterruptedException) {
            Log.e(TAG, "stopBackgroundThread", e)
        }
    }

    protected fun stopCamera() {
        if (camera != null) {
            camera!!.stopPreview()
            camera!!.setPreviewCallback(null)
            camera!!.release()
            camera = null
        }
    }

    private fun getCameraId(): Int {
        val ci = CameraInfo()
        for (i in 0 until getNumberOfCameras()) {
            getCameraInfo(i, ci)
            if (ci.facing == CameraInfo.CAMERA_FACING_BACK) return i
        }
        return -1 // No camera found
    }
}