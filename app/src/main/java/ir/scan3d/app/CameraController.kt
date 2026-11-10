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
import androidx.camera.core.resolutionselector.AspectRatioStrategy
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
 *   - exposure and white balance locked once the scene is framed; where the sensor can be driven by hand the
 *     lock also shortens the exposure to 1/100 s (ISO raised to match): hand shake smears a long exposure by
 *     several pixels, and a little more sensor noise costs far less accuracy than that
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
    var sensorOrientation = 90; private set
    var sensorSize: Size? = null; private set
    var locked = false; private set
    /** Shorten the exposure when locking (needs a sensor with manual control); set from the user's preference. */
    var shortExposure = true
    /** True while the lock holds a hand-set exposure (false: plain auto-exposure lock). */
    var manualExposure = false; private set
    private var manualOk = false; private var isoMax = 0
    var info: org.json.JSONObject? = null; private set     // lens / sensor facts for the session file

    fun start(owner: LifecycleOwner, previewView: PreviewView, analyzer: (ImageProxy) -> Unit, onReady: () -> Unit) {
        val future = ProcessCameraProvider.getInstance(ctx)
        future.addListener({
            val provider = future.get()
            // 4:3 like the sensor and the analysis frames: what is drawn over the preview lines up with the picture
            val preview = Preview.Builder().also { tuneRequest(it) }
                .setResolutionSelector(ResolutionSelector.Builder().setAspectRatioStrategy(AspectRatioStrategy.RATIO_4_3_FALLBACK_AUTO_STRATEGY).build()).build()
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
            info.getCameraCharacteristic(CameraCharacteristics.SENSOR_ORIENTATION)?.let { sensorOrientation = it }
            val fl = info.getCameraCharacteristic(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
            val ps = info.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
            val px = info.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE)
            val streams = info.getCameraCharacteristic(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
            val jpeg = streams?.getOutputSizes(ImageFormat.JPEG)?.maxByOrNull { it.width.toLong() * it.height }
            manualOk = info.getCameraCharacteristic(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES)?.contains(CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR) == true
            info.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE)?.let { isoMax = it.upper }
            sensorSize = jpeg ?: px
            if (fl != null && fl.isNotEmpty() && ps != null && px != null && sensorSize != null) {
                // focal length in pixels of the full-resolution JPEG (sensor crop may differ slightly; a guess is enough)
                focalPx = (fl[0] / ps.width * sensorSize!!.width).toDouble()
            }
            Log.i("Scan3D", "sensor $sensorSize focal ${fl?.toList()} -> fpx $focalPx")
            // everything the phone knows about this lens, kept with the session: a factory intrinsic calibration
            // (where the vendor provides one) is an independent check of the solved camera
            val j = org.json.JSONObject()
            fl?.let { j.put("focal_mm", org.json.JSONArray(it.map { v -> v.toDouble() })) }
            ps?.let { j.put("sensor_mm", org.json.JSONArray(listOf(it.width.toDouble(), it.height.toDouble()))) }
            px?.let { j.put("pixel_array", org.json.JSONArray(listOf(it.width, it.height))) }
            jpeg?.let { j.put("jpeg", org.json.JSONArray(listOf(it.width, it.height))) }
            info.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE)?.let { j.put("active_array", org.json.JSONArray(listOf(it.left, it.top, it.right, it.bottom))) }
            info.getCameraCharacteristic(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)?.let { j.put("intrinsic", org.json.JSONArray(it.map { v -> v.toDouble() })) }
            if (android.os.Build.VERSION.SDK_INT >= 28) info.getCameraCharacteristic(CameraCharacteristics.LENS_DISTORTION)?.let { j.put("distortion", org.json.JSONArray(it.map { v -> v.toDouble() })) }
            info.getCameraCharacteristic(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE)?.let { j.put("min_focus_diopters", it.toDouble()) }
            info.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE)?.let { j.put("timestamp_source", it) }
            info.getCameraCharacteristic(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL)?.let { j.put("hardware_level", it) }
            j.put("sensor_orientation", sensorOrientation).put("manual_sensor", manualOk).put("iso_max", isoMax)
            this.info = j
        } catch (e: Exception) { Log.w("Scan3D", "characteristics: $e") }
    }

    /**
     * Lock exposure and white balance where they are now, and the focus at `focusDiopters` (default: where the
     * autofocus stands now). Call once the board is framed. A session that is continued passes the focus of its
     * earlier photos, so all of them share one focal length.
     */
    fun lock(focusDiopters: Float? = null) {
        val cam = camera ?: return
        val c2 = Camera2CameraControl.from(cam.cameraControl)
        val b = CaptureRequestOptions.Builder()
            .setCaptureRequestOption(CaptureRequest.CONTROL_AWB_LOCK, true)
            .setCaptureRequestOption(CaptureRequest.CONTROL_AF_MODE, CameraMetadata.CONTROL_AF_MODE_OFF)
            .setCaptureRequestOption(CaptureRequest.LENS_FOCUS_DISTANCE, focusDiopters ?: lastFocus)
        val exp = lastExposureNs; val iso = lastIso
        if (shortExposure && manualOk && isoMax > 0 && iso > 0 && exp > MAX_EXPOSURE_NS) {
            // same brightness (exposure time x ISO), shorter exposure: ISO up, but not beyond what the sensor does cleanly
            val isoNew = Math.round(iso.toDouble() * exp / MAX_EXPOSURE_NS).toInt().coerceAtMost(minOf(isoMax, MAX_ISO)).coerceAtLeast(iso)
            val expNew = Math.round(exp.toDouble() * iso / isoNew)
            b.setCaptureRequestOption(CaptureRequest.CONTROL_AE_MODE, CameraMetadata.CONTROL_AE_MODE_OFF)
                .setCaptureRequestOption(CaptureRequest.SENSOR_EXPOSURE_TIME, expNew)
                .setCaptureRequestOption(CaptureRequest.SENSOR_SENSITIVITY, isoNew)
            manualExposure = true
            Log.i("Scan3D", "lock: exposure ${exp / 1000} us ISO $iso -> ${expNew / 1000} us ISO $isoNew")
        } else {
            b.setCaptureRequestOption(CaptureRequest.CONTROL_AE_LOCK, true)
            manualExposure = false
        }
        c2.setCaptureRequestOptions(b.build())
        locked = true
    }

    fun unlock() {
        if (!locked) return
        val cam = camera ?: return
        Camera2CameraControl.from(cam.cameraControl).setCaptureRequestOptions(CaptureRequestOptions.Builder()
            .setCaptureRequestOption(CaptureRequest.CONTROL_AE_MODE, CameraMetadata.CONTROL_AE_MODE_ON)
            .setCaptureRequestOption(CaptureRequest.CONTROL_AE_LOCK, false)
            .setCaptureRequestOption(CaptureRequest.CONTROL_AWB_LOCK, false)
            .setCaptureRequestOption(CaptureRequest.CONTROL_AF_MODE, CameraMetadata.CONTROL_AF_MODE_CONTINUOUS_PICTURE)
            .build())
        locked = false; manualExposure = false
    }

    companion object {
        /** 1/100 s: short enough for a steady hand (under a pixel of smear), and one full flicker period of 50 Hz mains lighting. */
        const val MAX_EXPOSURE_NS = 10_000_000L
        const val MAX_ISO = 3200
    }

    fun torch(on: Boolean) { camera?.cameraControl?.enableTorch(on) }
}
