package ir.scan3d.app

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.ImageFormat
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraMetadata
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.TotalCaptureResult
import android.util.Log
import android.util.Size
import androidx.camera.camera2.interop.Camera2CameraControl
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.camera2.interop.Camera2Interop
import androidx.camera.camera2.interop.CaptureRequestOptions
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.core.Camera
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageCapture
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import java.util.concurrent.Executors

/**
 * Metrology-minded camera setup, which is what a web page cannot do:
 *   - main (1x) lens only, no zoom
 *   - video/optical stabilisation OFF (it warps frames and changes the effective intrinsics)
 *   - exposure and white balance locked once the scene is framed
 *   - focus locked after the first scan photo (a changing focus changes the focal length)
 *   - every capture reports exposure time, ISO and focus distance
 *   - highest JPEG resolution the sensor offers
 */
@SuppressLint("UnsafeOptInUsageError")
@OptIn(ExperimentalCamera2Interop::class)
class CameraController(private val ctx: Context) {
    val analysisExecutor = Executors.newSingleThreadExecutor()
    private var camera: Camera? = null
    var imageCapture: ImageCapture? = null; private set
    var lastExposureNs = 0L; private set
    var lastIso = 0; private set
    var lastFocus = 0f; private set
    var focalPx: Double? = null; private set
    var sensorSize: Size? = null; private set
    var locked = false; private set

    fun start(owner: LifecycleOwner, previewView: PreviewView, analyzer: (ImageProxy) -> Unit, onReady: () -> Unit) {
        val future = ProcessCameraProvider.getInstance(ctx)
        future.addListener({
            val provider = future.get()
            val preview = Preview.Builder().also { tuneRequest(it) }.build()
            preview.surfaceProvider = previewView.surfaceProvider

            val capB = ImageCapture.Builder()
                .setCaptureMode(ImageCapture.CAPTURE_MODE_MAXIMIZE_QUALITY)
                .setJpegQuality(97)
                .setResolutionSelector(ResolutionSelector.Builder().setResolutionStrategy(ResolutionStrategy.HIGHEST_AVAILABLE_STRATEGY).build())
            tuneRequest(capB)
            Camera2Interop.Extender(capB).setSessionCaptureCallback(object : android.hardware.camera2.CameraCaptureSession.CaptureCallback() {
                override fun onCaptureCompleted(s: android.hardware.camera2.CameraCaptureSession, r: CaptureRequest, result: TotalCaptureResult) {
                    result.get(CaptureResult.SENSOR_EXPOSURE_TIME)?.let { lastExposureNs = it }
                    result.get(CaptureResult.SENSOR_SENSITIVITY)?.let { lastIso = it }
                    result.get(CaptureResult.LENS_FOCUS_DISTANCE)?.let { lastFocus = it }
                }
            })
            val capture = capB.build()

            val analysis = ImageAnalysis.Builder()
                .setResolutionSelector(ResolutionSelector.Builder().setResolutionStrategy(
                    ResolutionStrategy(Size(1280, 960), ResolutionStrategy.FALLBACK_RULE_CLOSEST_LOWER_THEN_HIGHER)).build())
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888)
                .build()
            analysis.setAnalyzer(analysisExecutor, analyzer)

            provider.unbindAll()
            val cam = provider.bindToLifecycle(owner, CameraSelector.DEFAULT_BACK_CAMERA, preview, capture, analysis)
            camera = cam; imageCapture = capture
            cam.cameraControl.setZoomRatio(1.0f)
            readCharacteristics(cam)
            onReady()
        }, ContextCompat.getMainExecutor(ctx))
    }

    private fun <T> tuneRequest(builder: androidx.camera.core.ExtendableBuilder<T>) {
        Camera2Interop.Extender(builder)
            .setCaptureRequestOption(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CameraMetadata.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
            .setCaptureRequestOption(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_OFF)
            .setCaptureRequestOption(CaptureRequest.NOISE_REDUCTION_MODE, CameraMetadata.NOISE_REDUCTION_MODE_FAST)
            .setCaptureRequestOption(CaptureRequest.EDGE_MODE, CameraMetadata.EDGE_MODE_OFF)
            .setCaptureRequestOption(CaptureRequest.CONTROL_SCENE_MODE, CameraMetadata.CONTROL_SCENE_MODE_DISABLED)
    }

    private fun readCharacteristics(cam: Camera) {
        try {
            val info = Camera2CameraInfo.from(cam.cameraInfo)
            val fl = info.getCameraCharacteristic(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
            val ps = info.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
            val px = info.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE)
            val streams = info.getCameraCharacteristic(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
            val jpeg = streams?.getOutputSizes(ImageFormat.JPEG)?.maxByOrNull { it.width.toLong() * it.height }
            sensorSize = jpeg ?: px
            if (fl != null && fl.isNotEmpty() && ps != null && px != null && sensorSize != null) {
                // focal length in pixels of the full-resolution JPEG (sensor crop may differ slightly; a guess is enough)
                focalPx = (fl[0] / ps.width * sensorSize!!.width).toDouble()
            }
            Log.i("Scan3D", "sensor $sensorSize focal ${fl?.toList()} -> fpx $focalPx")
        } catch (e: Exception) { Log.w("Scan3D", "characteristics: $e") }
    }

    /** Lock exposure, white balance and focus where they are now. Call once the board is framed. */
    fun lock() {
        val cam = camera ?: return
        val c2 = Camera2CameraControl.from(cam.cameraControl)
        c2.setCaptureRequestOptions(CaptureRequestOptions.Builder()
            .setCaptureRequestOption(CaptureRequest.CONTROL_AE_LOCK, true)
            .setCaptureRequestOption(CaptureRequest.CONTROL_AWB_LOCK, true)
            .setCaptureRequestOption(CaptureRequest.CONTROL_AF_MODE, CameraMetadata.CONTROL_AF_MODE_OFF)
            .setCaptureRequestOption(CaptureRequest.LENS_FOCUS_DISTANCE, lastFocus)
            .build())
        locked = true
    }

    fun unlock() {
        val cam = camera ?: return
        Camera2CameraControl.from(cam.cameraControl).setCaptureRequestOptions(CaptureRequestOptions.Builder()
            .setCaptureRequestOption(CaptureRequest.CONTROL_AE_LOCK, false)
            .setCaptureRequestOption(CaptureRequest.CONTROL_AWB_LOCK, false)
            .setCaptureRequestOption(CaptureRequest.CONTROL_AF_MODE, CameraMetadata.CONTROL_AF_MODE_CONTINUOUS_PICTURE)
            .build())
        locked = false
    }

    fun torch(on: Boolean) { camera?.cameraControl?.enableTorch(on) }
}
