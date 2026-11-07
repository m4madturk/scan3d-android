package ir.scan3d.app

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.BitmapFactory
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.view.View
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.core.ImageCapture
import androidx.camera.core.ImageCaptureException
import androidx.camera.core.ImageProxy
import androidx.core.content.ContextCompat
import androidx.exifinterface.media.ExifInterface
import androidx.lifecycle.lifecycleScope
import ir.scan3d.app.databinding.ActivityMainBinding
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.json.JSONObject
import org.opencv.android.OpenCVLoader
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.imgproc.Imgproc
import java.io.File
import kotlin.math.abs
import kotlin.math.sqrt

class MainActivity : AppCompatActivity(), SensorEventListener {
    private lateinit var b: ActivityMainBinding
    private lateinit var api: Api
    private lateinit var cam: CameraController
    private var session: Session? = null
    private var calib: CalibPlanner? = null
    private var mode = "scan"                 // scan | calib
    private var busy = false
    private var autoShutter = false
    private var stillFrames = 0
    private var gyroRate = 0f
    private var lastPreviewPose: Pose? = null
    private var previewScaleX = 1f; private var previewScaleY = 1f
    private var autoLockDone = false
    private val fa = { n: Int -> n.toString().map { ch -> if (ch.isDigit()) ('۰' + (ch - '0')) else ch }.joinToString("") }

    private val permission = registerForActivityResult(ActivityResultContracts.RequestPermission()) { ok ->
        if (ok) startCamera() else { toast(getString(R.string.need_camera)); finish() }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        b = ActivityMainBinding.inflate(layoutInflater); setContentView(b.root)
        api = Api(this); cam = CameraController(this)
        if (!OpenCVLoader.initLocal()) { toast("OpenCV failed to load"); finish(); return }
        if (!api.configured) startActivity(Intent(this, SettingsActivity::class.java))

        b.btnShot.setOnClickListener { capture() }
        b.btnAuto.setOnClickListener { autoShutter = !autoShutter; b.btnAuto.text = getString(if (autoShutter) R.string.auto_on else R.string.auto_off) }
        b.btnLock.setOnClickListener { if (cam.locked) { cam.unlock(); b.btnLock.text = getString(R.string.lock) } else { cam.lock(); b.btnLock.text = getString(R.string.unlock) } }
        b.btnTorch.setOnClickListener { torch = !torch; cam.torch(torch) }
        b.btnMode.setOnClickListener { switchMode(if (mode == "scan") "calib" else "scan") }
        b.btnSettings.setOnClickListener { startActivity(Intent(this, SettingsActivity::class.java)) }
        b.btnNew.setOnClickListener { newSession() }
        b.btnBuild.setOnClickListener { requestBuild() }
        b.btnResult.setOnClickListener { session?.let { startActivity(Intent(this, ResultActivity::class.java).putExtra("sid", it.sid)) } }
        b.modeSel.setOnCheckedChangeListener { _, id ->
            session?.mode = when (id) { R.id.mode8 -> "8"; R.id.modeFree -> "free"; else -> "18" }
            session?.shots?.forEach { it.slotKey = session!!.slotOf(it.pose) }
            pushSession(); refresh()
        }
        newSession()
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) startCamera()
        else permission.launch(Manifest.permission.CAMERA)
    }

    private var torch = false

    override fun onResume() {
        super.onResume()
        (getSystemService(SENSOR_SERVICE) as SensorManager).let { sm ->
            sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        }
    }

    override fun onPause() { super.onPause(); (getSystemService(SENSOR_SERVICE) as SensorManager).unregisterListener(this) }
    override fun onSensorChanged(e: SensorEvent) { gyroRate = sqrt(e.values[0] * e.values[0] + e.values[1] * e.values[1] + e.values[2] * e.values[2]) }
    override fun onAccuracyChanged(s: Sensor?, a: Int) {}

    private fun startCamera() {
        cam.start(this, b.preview, ::analyzePreview) { b.status.text = getString(R.string.ready) }
    }

    // ---------------- live preview analysis ----------------
    private fun analyzePreview(img: ImageProxy) {
        try {
            val w = img.width; val h = img.height
            val plane = img.planes[0]; val buf = plane.buffer; val stride = plane.rowStride
            val gray = Mat(h, w, CvType.CV_8UC1)
            if (stride == w) { val arr = ByteArray(w * h); buf.get(arr, 0, w * h); gray.put(0, 0, arr) }
            else { val row = ByteArray(w); for (y in 0 until h) { buf.position(y * stride); buf.get(row, 0, w); gray.put(y, 0, row) } }
            val rot = img.imageInfo.rotationDegrees
            val up = Mat()
            when (rot) { 90 -> org.opencv.core.Core.rotate(gray, up, org.opencv.core.Core.ROTATE_90_CLOCKWISE)
                180 -> org.opencv.core.Core.rotate(gray, up, org.opencv.core.Core.ROTATE_180)
                270 -> org.opencv.core.Core.rotate(gray, up, org.opencv.core.Core.ROTATE_90_COUNTERCLOCKWISE)
                else -> gray.copyTo(up) }
            val (ids, nBoard) = Detector.quick(up)
            val boardOk = nBoard >= 6
            val still = gyroRate < 0.08f
            stillFrames = if (still) stillFrames + 1 else 0
            runOnUiThread {
                b.overlay.boardOk = boardOk
                b.live.text = getString(R.string.live_fmt, fa(ids.size), fa(nBoard), if (still) "●" else "○")
                if (boardOk && !autoLockDone && mode == "scan" && cam.locked.not() && stillFrames > 15) {
                    // first steady framing of the board: lock exposure/WB/focus so every shot shares one camera
                    cam.lock(); autoLockDone = true; b.btnLock.text = getString(R.string.unlock); toast(getString(R.string.locked_auto))
                }
                b.overlay.invalidate()
            }
            if (autoShutter && !busy && boardOk && stillFrames > 12 && ids.isNotEmpty()) { stillFrames = 0; runOnUiThread { capture() } }
            gray.release(); up.release()
        } catch (e: Exception) { /* preview analysis is best effort */ }
        finally { img.close() }
    }

    // ---------------- capture ----------------
    private fun capture() {
        val ic = cam.imageCapture ?: return
        if (busy) return
        busy = true
        val s = session ?: return
        val n = (s.shots.maxOfOrNull { it.n } ?: 0) + 1
        val shot = Shot(n, "k" + System.currentTimeMillis().toString(36))
        val file = File(s.dir, "shot$n.jpg")
        b.status.text = getString(R.string.capturing)
        ic.takePicture(ImageCapture.OutputFileOptions.Builder(file).build(), ContextCompat.getMainExecutor(this),
            object : ImageCapture.OnImageSavedCallback {
                override fun onImageSaved(r: ImageCapture.OutputFileResults) { process(shot, file) }
                override fun onError(e: ImageCaptureException) { busy = false; toast("capture: ${e.message}") }
            })
    }

    private fun process(shot: Shot, file: File) {
        val s = session ?: return
        shot.exposureNs = cam.lastExposureNs; shot.iso = cam.lastIso; shot.focusDist = cam.lastFocus
        if (mode == "scan") { s.shots.add(shot); refresh() }
        lifecycleScope.launch(Dispatchers.Default) {
            try {
                // CameraX writes the sensor-frame JPEG with an EXIF orientation tag: read it, keep sensor frame
                val exif = ExifInterface(file.absolutePath)
                shot.orientation = exif.getAttributeInt(ExifInterface.TAG_ORIENTATION, 1)
                val gray = Imgcodecs.imread(file.absolutePath, Imgcodecs.IMREAD_GRAYSCALE)
                shot.w = gray.cols(); shot.h = gray.rows()
                shot.landscape = when (shot.orientation) { 6, 8 -> false; else -> true }   // phone sideways -> no rotation tag
                val a = Detector.analyze(gray, cam.focalPx?.let { it * gray.cols() / (cam.sensorSize?.width ?: gray.cols()) })
                gray.release()
                shot.stickers = a.stickers; shot.board = a.board; shot.sharp = a.sharpness; shot.millis = a.millis
                shot.pose = a.pose; shot.slotKey = s.slotOf(a.pose)
                // thumbnail
                val opts = BitmapFactory.Options().apply { inSampleSize = 16 }
                val bmp = BitmapFactory.decodeFile(file.absolutePath, opts)
                shot.thumb = File(s.dir, "thumb${shot.n}.jpg").also { f -> f.outputStream().use { bmp.compress(android.graphics.Bitmap.CompressFormat.JPEG, 60, it) } }
                shot.state = "done"
                withContext(Dispatchers.Main) { refresh() }
                if (mode == "calib") {
                    val v = calib?.add(a.board, shot.landscape, a.sharpness)
                    withContext(Dispatchers.Main) { toast(if (v == null) getString(R.string.calib_no_board) else getString(R.string.calib_added, fa(a.board?.ids?.size ?: 0), v.tiltDeg.toInt().toString())); refresh() }
                }
                upload(shot, file)
            } catch (e: Exception) {
                shot.state = "error"
                withContext(Dispatchers.Main) { toast("analyze: ${e.message}"); refresh() }
            } finally { busy = false }
        }
    }

    private suspend fun upload(shot: Shot, file: File) {
        if (!api.configured) return
        val s = session ?: return
        try {
            withContext(Dispatchers.Main) { b.status.text = getString(R.string.uploading, fa(shot.n)) }
            shot.assetId = api.uploadJpeg(file.readBytes())
            val col = if (mode == "calib") "calib" else "sessions"
            api.putDoc("$col/${s.sid}/shots/${shot.id}", shot.toJson())
            shot.sent = true
            withContext(Dispatchers.Main) { refresh(); b.status.text = getString(R.string.sent, fa(shot.n)) }
        } catch (e: Exception) {
            withContext(Dispatchers.Main) { b.status.text = getString(R.string.upload_failed, e.message ?: "") }
        }
    }

    // ---------------- session / modes ----------------
    private fun newSession() {
        val sid = (if (mode == "calib") "c" else "s") + System.currentTimeMillis().toString(36)
        session = Session(this, sid, if (mode == "calib") "calibration ${android.os.Build.MODEL}" else "scan", if (mode == "calib") "free" else "18")
        calib = if (mode == "calib") CalibPlanner(cam.sensorSize?.width ?: 4000, cam.sensorSize?.height ?: 3000) else null
        autoLockDone = false
        b.sid.text = "session $sid"
        pushSession(); refresh()
    }

    private fun switchMode(m: String) {
        mode = m
        b.btnMode.text = getString(if (m == "calib") R.string.to_scan else R.string.to_calib)
        b.modeSel.visibility = if (m == "calib") View.GONE else View.VISIBLE
        b.btnBuild.text = getString(if (m == "calib") R.string.compute_calib else R.string.build_model)
        if (m == "calib") cam.unlock() else cam.lock()
        newSession()
    }

    private fun pushSession() {
        val s = session ?: return
        if (!api.configured) return
        lifecycleScope.launch(Dispatchers.IO) {
            try { api.putDoc((if (mode == "calib") "calib/" else "sessions/") + s.sid, s.sessionDoc()) } catch (_: Exception) {}
        }
    }

    private fun requestBuild() {
        val s = session ?: return
        if (!api.configured) { toast(getString(R.string.set_server)); return }
        val done = s.shots.filter { it.state == "done" && it.sent }
        if (mode == "calib") {
            val st = calib?.status()
            if (st == null || st.total < 15) { toast(getString(R.string.calib_more)); return }
            lifecycleScope.launch(Dispatchers.IO) {
                try {
                    api.putDoc("calib/${s.sid}/status/request", JSONObject().put("at", System.currentTimeMillis()).put("device", android.os.Build.MODEL).put("shots", done.size))
                    withContext(Dispatchers.Main) { toast(getString(R.string.calib_requested)); startActivity(Intent(this@MainActivity, ResultActivity::class.java).putExtra("sid", s.sid).putExtra("calib", true)) }
                } catch (e: Exception) { withContext(Dispatchers.Main) { toast("request: ${e.message}") } }
            }
            return
        }
        if (done.size < 8) { toast(getString(R.string.need_shots, fa(done.size))); return }
        lifecycleScope.launch(Dispatchers.IO) {
            try {
                api.putDoc("sessions/${s.sid}/status/request", JSONObject().put("at", System.currentTimeMillis()).put("shots", done.size)
                    .put("photos", done.size).put("convex", b.convex.isChecked).put("name", s.name).put("device", android.os.Build.MODEL))
                withContext(Dispatchers.Main) { startActivity(Intent(this@MainActivity, ResultActivity::class.java).putExtra("sid", s.sid)) }
            } catch (e: Exception) { withContext(Dispatchers.Main) { toast("request: ${e.message}") } }
        }
    }

    // ---------------- UI ----------------
    private fun refresh() {
        val s = session ?: return
        if (mode == "calib") {
            val st = calib?.status()
            b.overlay.calibCells = calib?.cellCounts(); b.overlay.layout = null
            b.next.text = st?.next ?: ""
            b.count.text = getString(R.string.calib_count, fa(st?.total ?: 0), fa(st?.rolled ?: 0), fa(st?.cellsCovered ?: 0), fa(st?.cellsTotal ?: 24))
            b.proto.text = ""
            b.overlay.invalidate(); return
        }
        b.overlay.calibCells = null
        b.overlay.layout = s.layout(); b.overlay.taken = s.takenSlots(); b.overlay.next = s.nextSlot()
        val done = s.shots.filter { it.state == "done" }
        val taken = s.takenSlots().size
        val L = s.layout()
        b.count.text = if (L != null) getString(R.string.count_fmt, fa(taken), fa(L.az * L.bands.size)) else getString(R.string.count_free, fa(done.size))
        val rotated = done.count { it.landscape }; val low = done.count { it.pose != null && it.pose!!.el < 30 }
        b.proto.text = getString(R.string.proto_fmt, fa(done.size), fa(rotated), fa(low))
        val cov = s.coverage(); val weak = cov.filter { !it.ready }
        val nk = s.nextSlot()
        b.next.text = when {
            done.isNotEmpty() && cov.isNotEmpty() && weak.isEmpty() && cov.size >= 4 -> getString(R.string.enough)
            done.size >= 6 && rotated < 2 -> getString(R.string.need_rotated)
            done.size >= 6 && low < 4 -> getString(R.string.need_low)
            weak.isNotEmpty() && done.size >= 4 -> getString(R.string.weak_marker, weak[0].id.toString(), fa(weak[0].n))
            L == null -> getString(R.string.free_hint)
            nk != null -> { val (bb, aa) = nk.split("-").map { it.toInt() }; getString(R.string.next_fmt, L.clock[aa], L.bandName[bb]) }
            else -> getString(R.string.all_slots)
        }
        val last = done.lastOrNull()
        b.lastShot.text = last?.let { getString(R.string.last_fmt, fa(it.n), fa(it.stickers.size), fa(it.board?.ids?.size ?: 0), it.sharp.toInt().toString(), it.pose?.let { p -> "${p.az.toInt()}°/${p.el.toInt()}°" } ?: "-", if (it.sent) "✓" else "…") } ?: ""
        b.overlay.invalidate()
    }

    private fun toast(t: String) = Toast.makeText(this, t, Toast.LENGTH_SHORT).show()
}
