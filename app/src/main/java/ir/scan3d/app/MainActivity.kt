package ir.scan3d.app

import android.Manifest
import android.app.Activity
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.BitmapFactory
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Build
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
import org.opencv.android.OpenCVLoader
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs
import java.io.File
import kotlin.math.sqrt

/**
 * Capture screen. Everything else (solver, carving, STL) runs on the phone in BuildService;
 * sessions and camera profiles are plain files under the app's private storage (Store).
 */
class MainActivity : AppCompatActivity(), SensorEventListener {
    private lateinit var b: ActivityMainBinding
    private lateinit var cam: CameraController
    private lateinit var store: Store
    private var session: Session? = null
    private var calib: CalibPlanner? = null
    private var mode = "scan"                 // scan | calib
    private var busy = false
    private var autoShutter = false
    private var stillFrames = 0
    private var gyroRate = 0f
    private var autoLockDone = false
    private var torch = false
    private val fa = { n: Int -> Ui.fa(n) }

    private val permission = registerForActivityResult(ActivityResultContracts.RequestPermission()) { ok ->
        if (ok) startCamera() else { toast(getString(R.string.need_camera)); finish() }
    }
    private val notifPermission = registerForActivityResult(ActivityResultContracts.RequestPermission()) { }
    private val pickSession = registerForActivityResult(ActivityResultContracts.StartActivityForResult()) { r ->
        if (r.resultCode == Activity.RESULT_OK) r.data?.getStringExtra("sid")?.let { loadSession(it) }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Ui.edgeToEdge(this)
        b = ActivityMainBinding.inflate(layoutInflater); setContentView(b.root)
        Ui.marginBelowStatusBar(b.live)
        Ui.marginAboveNavBar(b.panel) { top -> b.overlay.topInset = top.toFloat(); b.overlay.invalidate() }
        cam = CameraController(this); store = Store(this)
        if (!OpenCVLoader.initLocal()) { toast("OpenCV failed to load"); finish(); return }

        b.btnShot.setOnClickListener { capture() }
        b.btnAuto.setOnClickListener { autoShutter = !autoShutter; b.btnAuto.text = getString(if (autoShutter) R.string.auto_on else R.string.auto_off) }
        b.btnLock.setOnClickListener { if (cam.locked) { cam.unlock(); b.btnLock.text = getString(R.string.lock) } else { cam.lock(); b.btnLock.text = getString(R.string.unlock) } }
        b.btnTorch.setOnClickListener { torch = !torch; cam.torch(torch); b.btnTorch.alpha = if (torch) 1f else 0.6f }
        b.btnMode.setOnClickListener { switchMode(if (mode == "scan") "calib" else "scan") }
        b.btnNew.setOnClickListener { newSession() }
        b.btnBuild.setOnClickListener { requestBuild() }
        b.btnResult.setOnClickListener { session?.let { startActivity(Intent(this, ResultActivity::class.java).putExtra("sid", it.sid).putExtra("calib", mode == "calib")) } }
        b.btnSessions.setOnClickListener { pickSession.launch(Intent(this, SessionsActivity::class.java)) }
        b.modeSel.setOnCheckedChangeListener { _, id ->
            val s = session ?: return@setOnCheckedChangeListener
            s.mode = when (id) { R.id.mode8 -> "8"; R.id.modeFree -> "free"; else -> "18" }
            s.shots.forEach { it.slotKey = s.slotOf(it.pose) }
            persist(); refresh()
        }
        b.btnTorch.alpha = 0.6f

        val sid = intent.getStringExtra("sid")
        if (intent.getStringExtra("mode") == "calib") mode = "calib"
        if (sid != null && store.load(sid) != null) loadSession(sid) else newSession()
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) startCamera()
        else permission.launch(Manifest.permission.CAMERA)
    }

    override fun onResume() {
        super.onResume()
        (getSystemService(SENSOR_SERVICE) as SensorManager).let { sm -> sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) } }
        b.profile.text = store.profileSummary()?.let { getString(R.string.profile_fmt, it) } ?: getString(R.string.no_profile)
        refresh()
    }

    override fun onPause() { super.onPause(); (getSystemService(SENSOR_SERVICE) as SensorManager).unregisterListener(this) }
    override fun onSensorChanged(e: SensorEvent) { gyroRate = sqrt(e.values[0] * e.values[0] + e.values[1] * e.values[1] + e.values[2] * e.values[2]) }
    override fun onAccuracyChanged(s: Sensor?, a: Int) {}

    private fun startCamera() { cam.start(this, b.preview, ::analyzePreview) { b.status.text = getString(R.string.ready) } }

    // ---------------- live preview analysis ----------------
    private fun analyzePreview(img: ImageProxy) {
        try {
            val w = img.width; val h = img.height
            val plane = img.planes[0]; val buf = plane.buffer; val stride = plane.rowStride
            val gray = Mat(h, w, CvType.CV_8UC1)
            if (stride == w) { val arr = ByteArray(w * h); buf.get(arr, 0, w * h); gray.put(0, 0, arr) }
            else { val row = ByteArray(w); for (y in 0 until h) { buf.position(y * stride); buf.get(row, 0, w); gray.put(y, 0, row) } }
            val up = Mat()
            when (img.imageInfo.rotationDegrees) {
                90 -> Core.rotate(gray, up, Core.ROTATE_90_CLOCKWISE); 180 -> Core.rotate(gray, up, Core.ROTATE_180)
                270 -> Core.rotate(gray, up, Core.ROTATE_90_COUNTERCLOCKWISE); else -> gray.copyTo(up)
            }
            val (ids, nBoard) = Detector.quick(up)
            val boardOk = nBoard >= 6
            val still = gyroRate < 0.08f
            stillFrames = if (still) stillFrames + 1 else 0
            runOnUiThread {
                b.overlay.boardOk = boardOk
                b.live.text = getString(R.string.live_fmt, fa(ids.size), fa(nBoard), if (still) "●" else "○")
                if (boardOk && !autoLockDone && mode == "scan" && !cam.locked && stillFrames > 15) {
                    // first steady framing of the board: lock exposure/WB/focus so every shot shares one camera
                    cam.lock(); autoLockDone = true; b.btnLock.text = getString(R.string.unlock); toast(getString(R.string.locked_auto))
                }
                b.overlay.invalidate()
            }
            if (autoShutter && !busy && boardOk && stillFrames > 12 && (ids.isNotEmpty() || mode == "calib")) { stillFrames = 0; runOnUiThread { capture() } }
            gray.release(); up.release()
        } catch (e: Exception) { /* preview analysis is best effort */ } finally { img.close() }
    }

    // ---------------- capture ----------------
    private fun capture() {
        val ic = cam.imageCapture ?: return
        if (busy) return
        busy = true
        val s = session ?: return
        val n = (s.shots.maxOfOrNull { it.n } ?: 0) + 1
        val shot = Shot(n, "k$n")
        val file = File(store.sessionDir(s.sid), "shot$n.jpg")
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
        s.shots.add(shot); refresh()
        lifecycleScope.launch(Dispatchers.Default) {
            try {
                // CameraX writes the sensor-frame JPEG with an EXIF orientation tag: read it, keep the sensor frame
                val exif = ExifInterface(file.absolutePath)
                shot.orientation = exif.getAttributeInt(ExifInterface.TAG_ORIENTATION, 1)
                val gray = Imgcodecs.imread(file.absolutePath, Imgcodecs.IMREAD_GRAYSCALE)
                shot.w = gray.cols(); shot.h = gray.rows()
                shot.landscape = shot.orientation !in intArrayOf(6, 8)   // phone sideways -> no 90 degree rotation tag
                val a = Detector.analyze(gray, cam.focalPx?.let { it * gray.cols() / (cam.sensorSize?.width ?: gray.cols()) })
                gray.release()
                shot.stickers = a.stickers; shot.board = a.board; shot.sharp = a.sharpness; shot.millis = a.millis
                shot.pose = a.pose; shot.slotKey = s.slotOf(a.pose)
                val opts = BitmapFactory.Options().apply { inSampleSize = 16 }
                BitmapFactory.decodeFile(file.absolutePath, opts)?.let { bmp ->
                    shot.thumb = File(store.sessionDir(s.sid), "thumb${shot.n}.jpg").also { f -> f.outputStream().use { bmp.compress(android.graphics.Bitmap.CompressFormat.JPEG, 60, it) } }
                }
                shot.state = "done"
                if (mode == "calib") {
                    val v = planner(shot.w, shot.h).add(a.board, shot.landscape, a.sharpness)
                    withContext(Dispatchers.Main) { toast(if (v == null) getString(R.string.calib_no_board) else getString(R.string.calib_added, fa(a.board?.ids?.size ?: 0), fa(v.tiltDeg.toInt()))) }
                }
                persist()
                withContext(Dispatchers.Main) { refresh(); b.status.text = getString(R.string.shot_done, fa(shot.n), fa(a.stickers.size), fa(a.board?.ids?.size ?: 0), a.millis.toString()) }
            } catch (e: Exception) {
                shot.state = "error"
                withContext(Dispatchers.Main) { toast("analyze: ${e.message}"); refresh() }
            } finally { busy = false }
        }
    }

    // ---------------- session / modes ----------------
    /** Calibration planner sized to the real sensor frame; rebuilt from the stored shots when the size changes. */
    private fun planner(w: Int, h: Int): CalibPlanner {
        val p = calib
        if (p != null && p.width == w && p.height == h) return p
        val np = CalibPlanner(w, h)
        session?.shots?.filter { it.state == "done" && it.w == w && it.h == h }?.forEach { np.add(it.board, it.landscape, it.sharp) }
        calib = np; return np
    }

    private fun newSession() {
        val sid = (if (mode == "calib") "c" else "s") + System.currentTimeMillis().toString(36)
        session = Session(this, sid, if (mode == "calib") "calibration" else "scan", if (mode == "calib") "free" else "18", mode == "calib")
        calib = null
        autoLockDone = false
        applyModeUi(); persist(); refresh()
    }

    private fun loadSession(sid: String) {
        val s = store.load(sid) ?: return
        session = s; mode = if (s.calib) "calib" else "scan"; calib = null
        if (s.calib) s.shots.firstOrNull { it.state == "done" }?.let { planner(it.w, it.h) }
        b.modeSel.check(when (s.mode) { "8" -> R.id.mode8; "free" -> R.id.modeFree; else -> R.id.mode18 })
        applyModeUi(); refresh()
    }

    private fun switchMode(m: String) { mode = m; if (m == "calib") cam.unlock() else if (autoLockDone) cam.lock(); newSession() }

    private fun applyModeUi() {
        b.btnMode.text = getString(if (mode == "calib") R.string.to_scan else R.string.to_calib)
        b.modeSel.visibility = if (mode == "calib") View.GONE else View.VISIBLE
        b.convex.visibility = if (mode == "calib") View.GONE else View.VISIBLE
        b.btnBuild.text = getString(if (mode == "calib") R.string.compute_calib else R.string.build_model)
        b.btnNew.text = getString(if (mode == "calib") R.string.new_calib else R.string.new_scan)
        b.sid.text = "session ${session?.sid ?: ""}"
    }

    private fun persist() { session?.let { s -> lifecycleScope.launch(Dispatchers.IO) { try { store.save(s) } catch (_: Exception) {} } } }

    private fun requestBuild() {
        val s = session ?: return
        val done = s.shots.filter { it.state == "done" }
        if (mode == "calib") {
            val st = calib?.status()
            if (st == null || st.total < 15) { toast(getString(R.string.calib_more)); return }
        } else if (done.size < 8) { toast(getString(R.string.need_shots, fa(done.size))); return }
        if (Build.VERSION.SDK_INT >= 33 && ContextCompat.checkSelfPermission(this, Manifest.permission.POST_NOTIFICATIONS) != PackageManager.PERMISSION_GRANTED)
            notifPermission.launch(Manifest.permission.POST_NOTIFICATIONS)   // progress notification; the build runs either way
        store.save(s)
        ContextCompat.startForegroundService(this, Intent(this, BuildService::class.java).putExtra("sid", s.sid).putExtra("calib", mode == "calib").putExtra("convex", b.convex.isChecked))
        startActivity(Intent(this, ResultActivity::class.java).putExtra("sid", s.sid).putExtra("calib", mode == "calib"))
    }

    // ---------------- UI ----------------
    private fun refresh() {
        val s = session ?: return
        if (mode == "calib") {
            val st = calib?.status()
            b.overlay.calibCells = calib?.cellCounts() ?: IntArray(24); b.overlay.layout = null
            b.next.text = st?.next ?: CalibPlanner(4000, 3000).status().next
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
            done.isNotEmpty() && cov.isNotEmpty() && weak.isEmpty() && cov.size >= 4 && rotated >= 2 && low >= 4 -> getString(R.string.enough)
            done.size >= 6 && rotated < 2 -> getString(R.string.need_rotated)
            done.size >= 6 && low < 4 -> getString(R.string.need_low)
            weak.isNotEmpty() && done.size >= 4 -> getString(R.string.weak_marker, weak[0].id.toString(), fa(weak[0].n))
            L == null -> getString(R.string.free_hint)
            nk != null -> { val (bb, aa) = nk.split("-").map { it.toInt() }; getString(R.string.next_fmt, L.clock[aa], L.bandName[bb]) }
            else -> getString(R.string.all_slots)
        }
        b.overlay.invalidate()
    }

    private fun toast(t: String) = Toast.makeText(this, t, Toast.LENGTH_SHORT).show()
}
