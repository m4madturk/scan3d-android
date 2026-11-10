package ir.scan3d.app

import android.Manifest
import android.app.Activity
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Matrix
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Build
import android.os.Bundle
import android.os.SystemClock
import android.text.SpannableString
import android.text.style.ForegroundColorSpan
import android.view.HapticFeedbackConstants
import android.view.KeyEvent
import android.view.View
import android.view.WindowManager
import android.widget.LinearLayout
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.core.ImageCapture
import androidx.camera.core.ImageCaptureException
import androidx.camera.core.ImageProxy
import androidx.core.content.ContextCompat
import androidx.exifinterface.media.ExifInterface
import androidx.lifecycle.lifecycleScope
import com.google.android.material.bottomsheet.BottomSheetBehavior
import com.google.android.material.bottomsheet.BottomSheetDialog
import ir.scan3d.app.databinding.ActivityMainBinding
import ir.scan3d.app.databinding.ItemShotBinding
import ir.scan3d.app.databinding.SheetMoreBinding
import ir.scan3d.app.databinding.SheetShotsBinding
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.opencv.android.OpenCVLoader
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs
import java.io.File
import java.util.concurrent.Executors
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Capture screen. The camera fills the screen; below it one instruction (what the next photo is and how to move),
 * what the camera sees right now, and one big shutter. Everything that is not needed for every photo lives in the
 * "more" sheet; the photos taken so far (with their quality) in the "photos" sheet.
 *
 * A photo is only exposed while the phone is still: the gyroscope gives the rotation rate, the camera its
 * exposure time, and their product is the smear in pixels. The shutter waits (briefly) until that is below a
 * pixel, and every photo keeps the smear it actually got.
 *
 * Solver, carving and STL run in BuildService; sessions and lens profiles are plain files (Store).
 */
class MainActivity : AppCompatActivity(), SensorEventListener {
    private lateinit var b: ActivityMainBinding
    private lateinit var cam: CameraController
    private lateinit var store: Store
    private val disk = Executors.newSingleThreadExecutor()        // session files: written in order, off the main thread
    private val prefs by lazy { getSharedPreferences("scan3d", MODE_PRIVATE) }
    private var session: Session? = null
    private var calib: CalibPlanner? = null
    private var mode = "scan"                 // scan | calib
    @Volatile private var busy = false        // a photo is being saved / measured
    private var waitingSince = 0L             // > 0 while the shutter waits for a steady hand
    private var autoShutter = false
    private var stillFrames = 0
    private var autoLockDone = false
    private var torch = false
    private var convex = true
    @Volatile private var livePose: Pose? = null
    private var demoPose: Pose? = null
    // IMU: elevation from gravity, heading from the rotation vector (anchored to the board's azimuth when it is seen)
    @Volatile private var gravEl: Double? = null
    @Volatile private var gravRoll = 0.0
    @Volatile private var yaw: Double? = null
    @Volatile private var gyroRate = 0f
    @Volatile private var lastShakeAt = 0L    // last time the phone turned faster than the blur budget allows
    private val gyroT = LongArray(256); private val gyroR = FloatArray(256); private var gyroN = 0
    private var azOffset: Double? = null
    private var lastDistMm: Double? = null
    private var poseFromBoard = false
    private var azKnown = false
    private val rotM = FloatArray(9)
    private var lastHint = 0L
    private var seenBoard = false; private var seenStickers = 0
    private val fa = { n: Int -> Ui.fa(n) }
    private fun clockOf(az: Double): String { val k = ((Math.round(az / 30.0).toInt() % 12) + 12) % 12; return Ui.fa(if (k == 0) 12 else k) }
    private fun col(id: Int) = ContextCompat.getColor(this, id)

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
        Ui.marginBelowStatusBar(b.topBar)
        Ui.marginAboveNavBar(b.panel)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        cam = CameraController(this); store = Store(this)
        if (!OpenCVLoader.initLocal()) { toast("OpenCV failed to load"); finish(); return }
        convex = prefs.getBoolean("convex", true)

        b.btnShot.setOnClickListener { shutter() }
        b.btnAuto.setOnClickListener { autoShutter = !autoShutter; b.btnAuto.isSelected = autoShutter; b.btnAuto.text = getString(if (autoShutter) R.string.auto_on else R.string.auto_off) }
        b.btnBuild.setOnClickListener { requestBuild() }
        b.btnShots.setOnClickListener { showShots() }
        b.btnMore.setOnClickListener { showMore() }
        b.modeChip.setOnClickListener { showMore() }
        b.btnHelp.setOnClickListener { openHelp(if (mode == "calib") 6 else 0) }
        b.guide.setOnClickListener { toggleGuideSize() }
        b.guide.setOnLongClickListener { openHelp(if (mode == "calib") 6 else 2); true }
        if (!prefs.getBoolean("tutorial_seen", false)) { prefs.edit().putBoolean("tutorial_seen", true).apply(); startActivity(Intent(this, TutorialActivity::class.java)) }
        b.guide.compact = true

        val sid = intent.getStringExtra("sid")
        if (intent.getStringExtra("mode") == "calib") mode = "calib"
        intent.getDoubleArrayExtra("demoPose")?.let { demoPose = Pose(it[0], it[1], it[2], DoubleArray(3), DoubleArray(3), it[3]) }   // screenshot tests only
        if (sid != null && store.exists(sid)) loadSession(sid) else if (mode != "scan" || !resumeLast()) newSession()
        renderChips()
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) startCamera()
        else permission.launch(Manifest.permission.CAMERA)
    }

    override fun onResume() {
        super.onResume()
        (getSystemService(SENSOR_SERVICE) as SensorManager).let { sm ->
            sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
            sm.getDefaultSensor(Sensor.TYPE_GRAVITY)?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
            (sm.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) ?: sm.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR))?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        }
        refresh()
        demoPose?.let { dp ->
            livePose = dp; seenBoard = true; seenStickers = 5
            b.guide.live = GuideView.PhonePose(dp.az, dp.el, dp.dist * 1000, dp.roll, 0.0, 0.0, if (mode == "calib") 0.0 else 15.0); b.guide.liveFresh = true
            b.live.text = getString(R.string.live_pose_fmt, clockOf(dp.az), fa(dp.el.toInt()), fa((dp.dist * 100).toInt()))
            renderChips(); updateHint()
        }
    }

    override fun onPause() {
        super.onPause()
        (getSystemService(SENSOR_SERVICE) as SensorManager).unregisterListener(this)
        if (waitingSince != 0L) { waitingSince = 0L; b.btnShot.state = ShutterView.State.READY }
    }

    override fun onDestroy() { super.onDestroy(); disk.shutdown() }

    /** The volume keys are a shutter too: easier than the screen when the phone is held low and sideways. */
    override fun onKeyDown(keyCode: Int, event: KeyEvent): Boolean {
        if (keyCode == KeyEvent.KEYCODE_VOLUME_DOWN || keyCode == KeyEvent.KEYCODE_VOLUME_UP) { if (event.repeatCount == 0) shutter(); return true }
        return super.onKeyDown(keyCode, event)
    }
    override fun onKeyUp(keyCode: Int, event: KeyEvent): Boolean =
        if (keyCode == KeyEvent.KEYCODE_VOLUME_DOWN || keyCode == KeyEvent.KEYCODE_VOLUME_UP) true else super.onKeyUp(keyCode, event)

    // ---------------- motion ----------------
    /** Exposure time of the frames the camera is producing now (s); 1/60 until the camera has reported one. */
    private fun exposureS(): Double = cam.lastExposureNs.let { if (it > 0) it / 1e9 else 1.0 / 60 }
    /** Focal length in pixels of the full-resolution photo. */
    private fun focalFullPx(): Double = cam.focalPx ?: 3000.0
    /** Rotation rate (rad/s) that smears the photo by BLUR_BUDGET_PX during one exposure; bounded to what a hand can do. */
    private fun steadyRate(): Float = (BLUR_BUDGET_PX / (exposureS() * focalFullPx())).coerceIn(0.015, 0.09).toFloat()

    override fun onSensorChanged(e: SensorEvent) {
        when (e.sensor.type) {
            Sensor.TYPE_GYROSCOPE -> {
                val r = sqrt(e.values[0] * e.values[0] + e.values[1] * e.values[1] + e.values[2] * e.values[2])
                val now = SystemClock.elapsedRealtime()
                gyroRate = r
                gyroT[gyroN and 255] = now; gyroR[gyroN and 255] = r; gyroN++
                if (r > steadyRate()) lastShakeAt = now
            }
            Sensor.TYPE_GRAVITY -> {
                // camera looks along -Z of the device; its angle below the horizon = asin(g_z / |g|)
                val n = sqrt(e.values[0] * e.values[0] + e.values[1] * e.values[1] + e.values[2] * e.values[2]); if (n < 1e-3) return
                gravEl = Math.toDegrees(Math.asin((e.values[2] / n).toDouble().coerceIn(-1.0, 1.0)))
                gravRoll = Math.toDegrees(Math.atan2(e.values[0].toDouble(), e.values[1].toDouble()))
            }
            Sensor.TYPE_GAME_ROTATION_VECTOR, Sensor.TYPE_ROTATION_VECTOR -> {
                SensorManager.getRotationMatrixFromVector(rotM, e.values)
                // camera axis (device -Z) in world coordinates = -(third column); heading = atan2(east, north)
                yaw = Math.toDegrees(Math.atan2(-rotM[2].toDouble(), -rotM[5].toDouble()))
            }
        }
    }
    override fun onAccuracyChanged(s: Sensor?, a: Int) {}

    /** Largest rotation rate between two moments (elapsedRealtime ms), or null when the gyroscope has no sample there. */
    private fun gyroPeak(from: Long, to: Long): Float? {
        var peak = -1f
        for (i in 0 until min(gyroN, 256)) if (gyroT[i] in from..to && gyroR[i] > peak) peak = gyroR[i]
        return if (peak < 0f) null else peak
    }

    /**
     * Live pose for the guide. The board gives the exact pose and teaches us the heading offset; without
     * the board we still know elevation (gravity), azimuth (heading + offset) and roughly the distance
     * (sticker size), so "higher / lower / closer" keeps working while the user moves.
     */
    private fun fuse(board: Pose?, stickerPx: Double?, fPrev: Double?): Pose? {
        if (board != null) {
            yaw?.let { azOffset = ((board.az - it) % 360 + 360) % 360 }
            lastDistMm = board.dist * 1000; poseFromBoard = true; azKnown = true
            return board
        }
        poseFromBoard = false
        val el = gravEl ?: return null
        val az = yaw?.let { y -> azOffset?.let { (((y + it) % 360) + 360) % 360 } }
        azKnown = az != null
        val dist = if (stickerPx != null && stickerPx > 8 && fPrev != null) fPrev * Detector.TARGET_MM / stickerPx else lastDistMm
        if (dist != null) lastDistMm = dist
        return Pose(az ?: (b.guide.target?.az ?: 180.0), el, (dist ?: 320.0) / 1000, DoubleArray(3), DoubleArray(3), gravRoll)
    }

    private fun startCamera() {
        cam.start(this, b.preview, ::analyzePreview) {
            showStatus(getString(R.string.ready))
            session?.let { if (it.camera == null) it.camera = cam.info }
        }
    }

    // ---------------- live preview analysis ----------------
    private var lastAnalysis = 0L
    private var lastBoardSeen = 0L

    private fun analyzePreview(img: ImageProxy) {
        val now = System.currentTimeMillis()
        // CPU budget: ~5 detections per second is plenty for guidance; skip blurred frames while the phone
        // turns fast (the IMU carries the pose), and everything while a photo is being processed
        if (busy || now - lastAnalysis < 200 || (gyroRate > 1.2f && now - lastBoardSeen < 1500)) { img.close(); return }
        lastAnalysis = now
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
            val fPrev = cam.focalPx?.let { f -> cam.sensorSize?.let { ss -> f * max(up.cols(), up.rows()) / max(ss.width, ss.height) } }
            val lv = Detector.live(up, fPrev)
            gray.release(); up.release()
            val boardOk = lv.nBoard >= 6
            livePose = demoPose ?: fuse(lv.pose, lv.stickerSidePx, fPrev)
            if (lv.pose != null) lastBoardSeen = now
            stillFrames = if (gyroRate < 0.08f) stillFrames + 1 else 0
            val steadyFor = SystemClock.elapsedRealtime() - lastShakeAt
            runOnUiThread { renderLive(lv, boardOk) }
            if (autoShutter && !busy && boardOk && steadyFor > AUTO_STEADY_MS && (lv.ids.isNotEmpty() || mode == "calib") && autoWanted(livePose, now))
                runOnUiThread { if (waitingSince == 0L) capture() }
        } catch (e: Exception) { /* preview analysis is best effort */ } finally { img.close() }
    }

    private fun renderLive(lv: Live, boardOk: Boolean) {
        if (isDestroyed) return
        seenBoard = boardOk || demoPose != null; seenStickers = if (demoPose != null) 5 else lv.ids.size
        b.overlay.boardOk = seenBoard; b.overlay.live = lv
        val lp = livePose
        b.overlay.current = if (lp != null && (poseFromBoard || demoPose != null)) session?.slotOf(lp) else null
        b.live.text = if (lp != null) (if (poseFromBoard || demoPose != null) "" else "≈ ") +
                getString(R.string.live_pose_fmt, if (azKnown || demoPose != null) clockOf(lp.az) else "؟", fa(lp.el.toInt()), fa((lp.dist * 100).toInt()))
            else getString(R.string.live_fmt, fa(lv.ids.size), fa(lv.nBoard))
        b.guide.live = lp?.let { GuideView.PhonePose(it.az, it.el, it.dist * 1000, it.roll, 0.0, 0.0, if (mode == "calib") 0.0 else 15.0) }
        b.guide.liveFresh = lp != null && (poseFromBoard || azKnown || demoPose != null)
        renderChips()
        if (System.currentTimeMillis() - lastHint > 400) { lastHint = System.currentTimeMillis(); updateHint() }
        if (boardOk && !autoLockDone && mode == "scan" && !cam.locked && stillFrames > 5) {
            // first steady framing of the board: lock exposure/WB/focus so every shot shares one camera
            cam.lock(sessionFocus()); autoLockDone = true; showStatus(getString(R.string.locked_auto)); renderChips()
        }
        b.overlay.invalidate()
    }

    /** One small chip per thing the camera needs: a coloured dot says whether it has it. */
    private fun renderChips() {
        val scan = mode == "scan"
        chip(b.chipBoard, if (seenBoard) R.color.ok else R.color.bad, getString(R.string.chip_board_ok))
        b.chipStickers.visibility = if (scan) View.VISIBLE else View.GONE
        chip(b.chipStickers, if (seenStickers > 0) R.color.ok else R.color.text_faint, getString(R.string.chip_stickers, fa(seenStickers)))
        val steady = SystemClock.elapsedRealtime() - lastShakeAt > STEADY_MS
        // in dim light the exposure is long and even a careful hand smears the picture: say so
        val dim = exposureS() > 1.0 / 45
        chip(b.chipSteady, if (steady) R.color.ok else R.color.warn, getString(if (steady) R.string.chip_steady else if (dim) R.string.chip_dim else R.string.chip_shaky))
        b.chipLock.visibility = if (scan) View.VISIBLE else View.GONE
        chip(b.chipLock, if (cam.locked) R.color.ok else R.color.text_faint, getString(R.string.chip_locked))
    }

    private fun chip(tv: TextView, colorRes: Int, text: String) {
        val sp = SpannableString("● $text"); sp.setSpan(ForegroundColorSpan(col(colorRes)), 0, 1, 0)
        tv.text = sp
    }

    /**
     * Focus distance (diopters) the earlier photos of this session were taken with, or null for a fresh session.
     * The focal length changes with the focus, and the solver assumes ONE camera per session: a scan that is
     * continued later is locked to the focus it started with.
     */
    private fun sessionFocus(): Float? {
        val f = session?.shots?.filter { it.state == "done" && it.focusDist > 0f }?.map { it.focusDist }?.sorted() ?: return null
        return if (f.isEmpty()) null else f[f.size / 2]
    }

    private var lastShotAt = 0L
    private var lastShotPose: Pose? = null

    /** Auto-shutter policy: not more than one photo per 2.5 s, only after the phone moved, and never a slot that is already taken. */
    private fun autoWanted(p: Pose?, now: Long): Boolean {
        if (now - lastShotAt < 2500) return false
        val s = session ?: return false
        if (p != null) {
            lastShotPose?.let { q -> val daz = abs(((p.az - q.az + 540) % 360) - 180); if (daz < 12 && abs(p.el - q.el) < 8 && abs(p.dist - q.dist) < 0.05) return false }
            if (mode == "scan" && s.layout() != null) { val k = s.slotOf(p); if (k != null && k in s.takenSlots()) return false }
        } else if (now - lastShotAt < 6000) return false
        return true
    }

    // ---------------- capture ----------------
    private val steadyWait = object : Runnable {
        override fun run() {
            if (waitingSince == 0L) return
            val now = SystemClock.elapsedRealtime()
            if (now - lastShakeAt >= STEADY_MS || now - waitingSince >= STEADY_TIMEOUT_MS) { waitingSince = 0L; capture() }
            else b.btnShot.postDelayed(this, 16)
        }
    }

    /** Shutter pressed: expose now if the phone is still, otherwise at the first still moment (at most 1.5 s later). */
    private fun shutter() {
        if (busy || waitingSince != 0L || session == null || cam.imageCapture == null) return
        val now = SystemClock.elapsedRealtime()
        if (now - lastShakeAt >= STEADY_MS) { capture(); return }
        waitingSince = now; b.btnShot.state = ShutterView.State.STEADY
        showStatus(getString(R.string.steadying), sticky = true)
        b.btnShot.post(steadyWait)
    }

    private fun capture() {
        val ic = cam.imageCapture ?: return
        val s = session ?: return
        if (busy) return
        busy = true; b.btnShot.state = ShutterView.State.BUSY
        val n = (s.shots.maxOfOrNull { it.n } ?: 0) + 1
        val shot = Shot(n, "k$n")
        shot.devRoll = gravRoll                       // how the phone was held: decides how the HAL may have rotated the JPEG
        lastShotAt = System.currentTimeMillis(); lastShotPose = livePose
        val file = File(store.sessionDir(s.sid), "shot$n.jpg")
        val called = SystemClock.elapsedRealtime(); var exposed = 0L
        val expS = exposureS(); val fFull = focalFullPx()
        b.btnShot.performHapticFeedback(HapticFeedbackConstants.KEYBOARD_TAP)
        showStatus(getString(R.string.capturing), sticky = true)
        ic.takePicture(ImageCapture.OutputFileOptions.Builder(file).build(), ContextCompat.getMainExecutor(this),
            object : ImageCapture.OnImageSavedCallback {
                override fun onCaptureStarted() { exposed = SystemClock.elapsedRealtime() }
                override fun onImageSaved(r: ImageCapture.OutputFileResults) {
                    // how fast the phone was turning while the frame was exposed, and what that smears
                    val expMs = (expS * 1000).toLong()
                    val peak = if (exposed > 0) gyroPeak(exposed - 80, exposed + expMs + 80) else gyroPeak(called, called + 600)
                    if (peak != null) { shot.motion = peak; shot.blurPx = peak * expS * fFull }
                    process(s, shot, file)
                }
                override fun onError(e: ImageCaptureException) {
                    busy = false; b.btnShot.state = ShutterView.State.READY
                    showStatus(getString(R.string.shot_failed, e.message ?: ""), R.color.bad)
                }
            })
    }

    private fun process(s: Session, shot: Shot, file: File) {
        shot.exposureNs = cam.lastExposureNs; shot.iso = cam.lastIso; shot.focusDist = cam.lastFocus
        s.shots.add(shot); refresh()
        lifecycleScope.launch {
            val err = withContext(Dispatchers.Default) {
                try {
                    // CameraX writes the sensor-frame JPEG with an EXIF orientation tag: read it, keep the sensor frame
                    val exif = ExifInterface(file.absolutePath)
                    shot.orientation = exif.getAttributeInt(ExifInterface.TAG_ORIENTATION, 1)
                    // IMREAD_IGNORE_ORIENTATION: OpenCV would otherwise apply the EXIF rotation, while the silhouette
                    // loader (BitmapFactory) does not -> detections and silhouettes in different frames (the slab bug)
                    val raw = Imgcodecs.imread(file.absolutePath, Imgcodecs.IMREAD_GRAYSCALE or Imgcodecs.IMREAD_IGNORE_ORIENTATION)
                    if (raw.empty()) throw IllegalStateException("photo could not be read")
                    val gray = toSensorFrame(raw, shot)
                    shot.w = gray.cols(); shot.h = gray.rows()
                    val a = Detector.analyze(gray, cam.focalPx?.let { it * gray.cols() / (cam.sensorSize?.width ?: gray.cols()) })
                    gray.release()
                    shot.stickers = a.stickers; shot.board = a.board; shot.sharp = a.sharpness; shot.millis = a.millis
                    shot.pose = a.pose
                    // thumbnail, upright the way the phone was held
                    val opts = BitmapFactory.Options().apply { inSampleSize = 8 }
                    BitmapFactory.decodeFile(file.absolutePath, opts)?.let { bmp ->
                        val deg = when (shot.orientation) { ExifInterface.ORIENTATION_ROTATE_90 -> 90f; ExifInterface.ORIENTATION_ROTATE_180 -> 180f; ExifInterface.ORIENTATION_ROTATE_270 -> 270f; else -> 0f }
                        val up = if (deg == 0f) bmp else Bitmap.createBitmap(bmp, 0, 0, bmp.width, bmp.height, Matrix().apply { postRotate(deg) }, true)
                        shot.thumb = File(store.sessionDir(s.sid), "thumb${shot.n}.jpg").also { f -> f.outputStream().use { up.compress(Bitmap.CompressFormat.JPEG, 72, it) } }
                        if (up !== bmp) up.recycle(); bmp.recycle()
                    }
                    null
                } catch (e: Throwable) { e }
            }
            // back on the main thread: the session is only ever changed here
            if (err != null) {
                s.shots.remove(shot); file.delete()
                showStatus(getString(R.string.shot_failed, err.message ?: err.toString()), R.color.bad)
            } else {
                shot.slotKey = s.slotOf(shot.pose)
                shot.pose?.let { p -> yaw?.let { azOffset = ((p.az - it) % 360 + 360) % 360; azKnown = true } }
                shot.state = "done"
                Session.markRolled(s.shots)                              // "rolled" = roll differs ~90 deg from the majority
                thumbShown = null
                if (session !== s) {                                     // the user moved on to another session meanwhile
                    showStatus(getString(R.string.shot_done, fa(shot.n), fa(shot.stickers.size), fa(shot.board?.ids?.size ?: 0)))
                    val text = store.doc(s); disk.execute { try { store.write(s.sid, text) } catch (_: Exception) {} }
                } else if (s.calib) {
                    calib = null                                          // rolled flags may have changed for earlier shots: rebuild
                    val pl = planner(shot.w, shot.h)
                    val v = if ((shot.board?.ids?.size ?: 0) >= 12) pl.views.lastOrNull() else null
                    if (v == null) showStatus(getString(R.string.calib_no_board), R.color.warn)
                    else showStatus(getString(R.string.calib_added, fa(shot.board?.ids?.size ?: 0), fa(v.tiltDeg.toInt())), R.color.ok)
                } else when (quality(shot, s)) {
                    Quality.NO_BOARD -> showStatus(getString(R.string.shot_no_board, fa(shot.n)), R.color.bad)
                    Quality.BLUR -> showStatus(getString(R.string.shot_blur, fa(shot.n), Ui.fa("%.1f".format(shot.blurPx))), R.color.bad)
                    Quality.SOFT -> showStatus(getString(R.string.shot_soft, fa(shot.n)), R.color.warn)
                    Quality.GOOD -> showStatus(getString(R.string.shot_done, fa(shot.n), fa(shot.stickers.size), fa(shot.board?.ids?.size ?: 0)), R.color.ok)
                }
                if (session === s) { persist(); refresh() }
            }
            busy = false; b.btnShot.state = ShutterView.State.READY
        }
    }

    enum class Quality { GOOD, SOFT, BLUR, NO_BOARD }

    /**
     * How usable a photo is. Motion smear comes from the gyroscope (physics, not a guess); "soft" also covers a
     * photo whose detail is far below the session's typical one (missed focus).
     */
    private fun quality(shot: Shot, s: Session): Quality {
        if ((shot.board?.ids?.size ?: 0) < 6 || shot.pose == null) return Quality.NO_BOARD
        if (shot.blurPx > 2.0) return Quality.BLUR
        val sharps = s.shots.filter { it.state == "done" && it.sharp > 0 }.map { it.sharp }.sorted()
        val soft = sharps.size >= 4 && shot.sharp < 0.35 * sharps[sharps.size / 2]
        return if (shot.blurPx > 1.0 || soft) Quality.SOFT else Quality.GOOD
    }

    /**
     * Some phones (Samsung among them) rotate the JPEG pixels to the way the phone was held, others store the
     * raw sensor frame and only tag EXIF. Everything downstream (lens profile, solver, silhouettes) wants ONE
     * frame, so the stored pixels are rotated back to the sensor frame here, using the gravity roll at capture.
     */
    private fun toSensorFrame(raw: Mat, shot: Shot): Mat {
        val portraitBuf = raw.rows() > raw.cols()
        if (portraitBuf && !prefs.getBoolean("hal_rotates_v2", false)) prefs.edit().putBoolean("hal_rotates_v2", true).apply()
        val halRotates = portraitBuf || prefs.getBoolean("hal_rotates_v2", false)
        var rot = 0
        if (halRotates) {
            val r = shot.devRoll
            val devRot = when { abs(r) < 45 -> 0; r >= 45 && r < 135 -> 90; r <= -45 && r > -135 -> 270; else -> 180 }   // Surface.ROTATION_x in degrees
            rot = ((devRot - cam.sensorOrientation) % 360 + 360) % 360           // undo "sensor rotated by (sensorOrientation - devRot)"
            val swaps = rot == 90 || rot == 270
            if (swaps != portraitBuf) rot = if (portraitBuf) ((0 - cam.sensorOrientation) % 360 + 360) % 360 else 0   // dims disagree with gravity: trust dims
        }
        shot.fileRot = rot
        if (rot == 0) return raw
        val out = Mat()
        Core.rotate(raw, out, when (rot) { 90 -> Core.ROTATE_90_CLOCKWISE; 180 -> Core.ROTATE_180; else -> Core.ROTATE_90_COUNTERCLOCKWISE })
        raw.release(); return out
    }

    // ---------------- session / modes ----------------
    /** Calibration planner sized to the real sensor frame; rebuilt from the stored shots when the size changes. */
    private fun planner(w: Int, h: Int): CalibPlanner {
        val p = calib
        if (p != null && p.width == w && p.height == h) return p
        val np = CalibPlanner(w, h)
        session?.shots?.filter { it.state == "done" && it.w == w && it.h == h }?.forEach { np.add(it.board, it.landscape, it.sharp, it.pose) }
        calib = np; return np
    }

    /** A session only reaches the disk (and the sessions list) with its first photo. */
    private fun newSession() {
        val sid = (if (mode == "calib") "c" else "s") + System.currentTimeMillis().toString(36)
        session = Session(this, sid, if (mode == "calib") "calibration" else "scan", if (mode == "calib") "free" else "18", mode == "calib").also { it.camera = cam.info }
        calib = null; thumbShown = null
        cam.unlock(); autoLockDone = false          // a new scene: exposure and focus are locked again at its first steady framing
        applyModeUi(); refresh()
    }

    private fun loadSession(sid: String) {
        val s = try { store.load(sid) } catch (_: Exception) { null } ?: return
        session = s; mode = if (s.calib) "calib" else "scan"; calib = null; thumbShown = null
        if (s.calib) s.shots.firstOrNull { it.state == "done" }?.let { planner(it.w, it.h) }
        prefs.edit().putString("last_sid", sid).apply()
        cam.unlock(); autoLockDone = false
        applyModeUi(); refresh()
    }

    /** Opening the app continues the scan that was in progress (not built yet, less than half a day old). */
    private fun resumeLast(): Boolean {
        val sid = prefs.getString("last_sid", null) ?: return false
        if (!store.exists(sid) || store.resultState(sid) == "done") return false
        val s = try { store.load(sid) } catch (_: Exception) { null } ?: return false
        if (s.calib || System.currentTimeMillis() - s.created > 12 * 3600_000L) return false
        loadSession(sid); return true
    }

    private fun switchMode(m: String) {
        mode = m
        if (m != "scan" || !resumeLast()) newSession()
        renderChips()
    }

    private fun applyModeUi() {
        b.modeChip.text = getString(if (mode == "calib") R.string.mode_calib else R.string.mode_scan)
        b.modeChip.setTextColor(col(if (mode == "calib") R.color.warn else R.color.text))
    }

    /** Writes the session file. The text is built here (main thread owns the session), the disk write is queued. */
    private fun persist() {
        val s = session ?: return
        if (s.shots.none { it.state == "done" } && !store.exists(s.sid)) return
        val text = store.doc(s); val sid = s.sid
        disk.execute { try { store.write(sid, text) } catch (_: Exception) {} }
        prefs.edit().putString("last_sid", sid).apply()
    }

    private fun requestBuild(force: Boolean = false) {
        val s = session ?: return
        if (busy) return
        BuildService.runningSid?.let { running ->          // one build at a time: it needs the phone's memory to itself
            if (running == s.sid) openResult() else showStatus(getString(R.string.build_running), R.color.warn)
            return
        }
        val done = s.shots.filter { it.state == "done" }
        if (mode == "calib") {
            val st = calib?.status()
            if (st == null || st.total < 15) { showStatus(getString(R.string.calib_more), R.color.warn); return }
        } else if (done.size < 8) { showStatus(getString(R.string.need_shots, fa(done.size)), R.color.warn); return }
        else if (!force) {
            // no low-elevation shot at all: the side walls and the base come out bloated; say so before building
            val lowBand = s.layout()?.bands?.firstOrNull()?.second ?: 35.0
            val lows = done.count { (it.pose?.el ?: 90.0) < lowBand }
            if (lows == 0) {
                AlertDialog.Builder(this).setMessage(getString(R.string.low_missing))
                    .setPositiveButton(getString(R.string.low_missing_go), null)
                    .setNegativeButton(getString(R.string.low_missing_anyway)) { _, _ -> requestBuild(force = true) }.show()
                return
            }
        }
        if (Build.VERSION.SDK_INT >= 33 && ContextCompat.checkSelfPermission(this, Manifest.permission.POST_NOTIFICATIONS) != PackageManager.PERMISSION_GRANTED)
            notifPermission.launch(Manifest.permission.POST_NOTIFICATIONS)   // progress notification; the build runs either way
        val text = store.doc(s); val sid = s.sid; val isCalib = mode == "calib"
        // the service reads the session file: start it only after this (and every queued) write has landed
        disk.execute {
            try { store.write(sid, text); File(store.sessionDir(sid), "result.json").delete() } catch (_: Exception) {}
            runOnUiThread {
                if (isDestroyed) return@runOnUiThread
                ContextCompat.startForegroundService(this, Intent(this, BuildService::class.java).putExtra("sid", sid).putExtra("calib", isCalib).putExtra("convex", convex))
                openResult()
            }
        }
    }

    private fun openResult() { session?.let { startActivity(Intent(this, ResultActivity::class.java).putExtra("sid", it.sid).putExtra("calib", it.calib)) } }
    private fun openHelp(page: Int) = startActivity(Intent(this, TutorialActivity::class.java).putExtra("page", page))

    // ---------------- sheets ----------------
    /** Everything that is not needed for every photo. */
    private fun showMore() {
        val s = session ?: return
        val dlg = BottomSheetDialog(this); val v = SheetMoreBinding.inflate(layoutInflater); dlg.setContentView(v.root)
        dlg.behavior.skipCollapsed = true; dlg.behavior.state = BottomSheetBehavior.STATE_EXPANDED
        val scan = mode == "scan"; val vis = { on: Boolean -> if (on) View.VISIBLE else View.GONE }
        v.lblAngles.visibility = vis(scan); v.angleRow.visibility = vis(scan); v.swConvex.visibility = vis(scan); v.swLock.visibility = vis(scan)
        fun mark() { v.mode18.isSelected = s.mode == "18"; v.mode8.isSelected = s.mode == "8"; v.modeFree.isSelected = s.mode == "free" }
        fun pick(m: String) { s.mode = m; s.shots.forEach { it.slotKey = s.slotOf(it.pose) }; mark(); persist(); refresh() }
        mark()
        v.mode18.setOnClickListener { pick("18") }; v.mode8.setOnClickListener { pick("8") }; v.modeFree.setOnClickListener { pick("free") }
        v.swLock.isChecked = cam.locked
        v.swLock.setOnCheckedChangeListener { _, on -> if (on) cam.lock(sessionFocus()) else cam.unlock(); autoLockDone = true; renderChips() }
        v.swTorch.isChecked = torch
        v.swTorch.setOnCheckedChangeListener { _, on -> torch = on; cam.torch(on) }
        v.swConvex.isChecked = convex
        v.swConvex.setOnCheckedChangeListener { _, on -> convex = on; prefs.edit().putBoolean("convex", on).apply() }
        v.profile.text = store.profileSummary()?.let { getString(R.string.profile_fmt, it) } ?: getString(R.string.no_profile)
        v.rowResult.visibility = vis(store.resultState(s.sid) != null)
        v.rowResult.setOnClickListener { dlg.dismiss(); openResult() }
        v.rowNew.text = getString(if (scan) R.string.new_scan else R.string.new_calib)
        v.rowNew.setOnClickListener {
            dlg.dismiss()
            if (s.shots.none { it.state == "done" }) return@setOnClickListener        // already empty
            AlertDialog.Builder(this).setMessage(getString(R.string.new_scan_q))
                .setPositiveButton(getString(R.string.yes)) { _, _ -> newSession() }.setNegativeButton(getString(R.string.no), null).show()
        }
        v.rowSessions.setOnClickListener { dlg.dismiss(); pickSession.launch(Intent(this, SessionsActivity::class.java)) }
        v.rowMode.text = getString(if (scan) R.string.to_calib else R.string.to_scan)
        v.rowMode.setOnClickListener { dlg.dismiss(); switchMode(if (scan) "calib" else "scan") }
        v.rowHelp.setOnClickListener { dlg.dismiss(); openHelp(if (scan) 0 else 6) }
        v.sid.text = s.sid
        dlg.show()
    }

    /** The photos of this session: what was measured in each, and a way to throw the bad ones away. */
    private fun showShots() {
        val s = session ?: return
        val dlg = BottomSheetDialog(this); val v = SheetShotsBinding.inflate(layoutInflater); dlg.setContentView(v.root)
        dlg.behavior.skipCollapsed = true; dlg.behavior.state = BottomSheetBehavior.STATE_EXPANDED
        fun fill() {
            v.shotsRow.removeAllViews()
            val done = s.shots.filter { it.state == "done" }.sortedByDescending { it.n }
            v.shotsEmpty.visibility = if (done.isEmpty()) View.VISIBLE else View.GONE
            v.shotsHint.visibility = if (done.isEmpty()) View.GONE else View.VISIBLE
            v.shotsTitle.text = getString(R.string.shots_title) + " · " + fa(done.size)
            for (sh in done) {
                val row = ItemShotBinding.inflate(layoutInflater, v.shotsRow, false)
                sh.thumb?.takeIf { f -> f.exists() }?.let { f -> row.thumb.setImageBitmap(BitmapFactory.decodeFile(f.absolutePath)) }
                val q = if (s.calib) (if ((sh.board?.ids?.size ?: 0) >= 12) Quality.GOOD else Quality.NO_BOARD) else quality(sh, s)
                val where = sh.pose?.let { p -> " · " + getString(R.string.shot_where, clockOf(p.az), fa(p.el.toInt())) } ?: ""
                row.title.text = getString(R.string.shot_title, fa(sh.n)) + where
                val qText = getString(when (q) { Quality.GOOD -> R.string.shot_q_good; Quality.SOFT -> R.string.shot_q_soft; Quality.BLUR -> R.string.shot_q_blur; Quality.NO_BOARD -> R.string.shot_q_noboard })
                val sp = SpannableString(qText + (if (sh.landscape) " · " + getString(R.string.shot_rolled) else "") + "\n" +
                        getString(R.string.shot_detail, fa(sh.stickers.size), fa(sh.board?.ids?.size ?: 0)))
                sp.setSpan(ForegroundColorSpan(col(when (q) { Quality.GOOD -> R.color.ok; Quality.SOFT -> R.color.warn; else -> R.color.bad })), 0, qText.length, 0)
                row.detail.text = sp
                row.delete.setOnClickListener { deleteShot(s, sh); fill() }
                v.shotsRow.addView(row.root)
            }
        }
        fill(); dlg.show()
    }

    private fun deleteShot(s: Session, sh: Shot) {
        s.shots.remove(sh)
        val dir = store.sessionDir(s.sid)
        disk.execute { File(dir, "shot${sh.n}.jpg").delete(); File(dir, "thumb${sh.n}.jpg").delete() }
        Session.markRolled(s.shots); calib = null; thumbShown = null
        if (s.calib) s.shots.firstOrNull { it.state == "done" }?.let { planner(it.w, it.h) }
        persist(); refresh()
    }

    // ---------------- UI ----------------
    private var thumbShown: String? = null

    private fun refresh() {
        val s = session ?: return
        val done = s.shots.filter { it.state == "done" }
        // photos so far
        b.shotCount.text = getString(R.string.shots_n, fa(done.size))
        val last = done.maxByOrNull { it.n }?.thumb?.takeIf { it.exists() }
        if (last?.absolutePath != thumbShown) { thumbShown = last?.absolutePath; b.lastThumb.setImageBitmap(last?.let { BitmapFactory.decodeFile(it.absolutePath) }) }
        if (mode == "calib") {
            val pl = calib ?: CalibPlanner(4000, 3000); val st = pl.status()
            b.overlay.calibCells = calib?.cellCounts() ?: IntArray(24); b.overlay.layout = null
            b.guide.mode = GuideView.Mode.CALIB; b.guide.slots = emptyMap(); b.guide.keyframes = emptyList()
            b.guide.target = GuideView.calibPose(st.target)
            b.count.text = getString(R.string.calib_count, fa(st.total), fa(st.rolled), fa(st.cellsCovered), fa(st.cellsTotal))
            b.btnBuild.text = getString(R.string.compute_calib_n, fa(st.total)); b.btnBuild.alpha = if (st.total >= 15) 1f else 0.45f
            updateHint()
            b.overlay.invalidate(); return
        }
        b.overlay.calibCells = null
        b.overlay.layout = s.layout(); b.overlay.taken = s.takenSlots(); b.overlay.next = s.nextSlot()
        b.guide.mode = GuideView.Mode.SCAN; b.guide.keyframes = emptyList()
        b.guide.setSlots(s.layout(), s.takenSlots(), s.nextSlot())
        b.guide.target = s.layout()?.let { L -> s.nextSlot()?.let { GuideView.slotPose(L, it) } }
        val taken = s.takenSlots().size
        val L = s.layout()
        b.count.text = if (L != null) getString(R.string.count_fmt, fa(taken), fa(L.az * L.bands.size)) else getString(R.string.count_free, fa(done.size))
        b.btnBuild.text = getString(R.string.build_model_n, fa(done.size)); b.btnBuild.alpha = if (done.size >= 8) 1f else 0.45f
        val rotated = done.count { it.landscape }; val low = done.count { it.pose != null && it.pose!!.el < 30 }
        val cov = s.coverage(); val weak = cov.filter { !it.ready }
        val nk = s.nextSlot()
        scanBase = when {
            done.isNotEmpty() && cov.isNotEmpty() && weak.isEmpty() && cov.size >= 4 && rotated >= 2 && low >= 4 -> getString(R.string.enough)
            done.size >= 6 && rotated < 2 -> { b.guide.target = L?.let { GuideView.slotPose(it, nk ?: "1-3").copy(roll = 90.0) }; getString(R.string.need_rotated) }
            done.size >= 6 && low < 4 -> { b.guide.target = L?.let { GuideView.slotPose(it, "0-" + ((nk?.split("-")?.get(1)) ?: "3")) }; getString(R.string.need_low) }
            weak.isNotEmpty() && done.size >= 4 -> getString(R.string.weak_marker, weak[0].id.toString(), fa(weak[0].n))
            L == null -> getString(R.string.free_hint)
            nk != null -> { val (bb, aa) = nk.split("-").map { it.toInt() }; getString(R.string.next_fmt, L.clock[aa], L.bandName[bb]) }
            else -> getString(R.string.all_slots)
        }
        updateHint()
        b.overlay.invalidate()
    }

    private var scanBase = ""
    private var guideLarge = false

    /** Tap on the little 3D guide: enlarge it to the full panel width (text moves below), tap again to shrink. */
    private fun toggleGuideSize() {
        guideLarge = !guideLarge
        val lp = b.guide.layoutParams as LinearLayout.LayoutParams
        if (guideLarge) {
            b.hintBox.orientation = LinearLayout.VERTICAL
            lp.width = LinearLayout.LayoutParams.MATCH_PARENT; lp.height = Ui.dp(this, 230)
            b.guide.compact = false
        } else {
            b.hintBox.orientation = LinearLayout.HORIZONTAL
            lp.width = Ui.dp(this, 96); lp.height = Ui.dp(this, 96)
            b.guide.compact = true
        }
        (b.hintTexts.layoutParams as LinearLayout.LayoutParams).let { t ->
            t.width = if (guideLarge) LinearLayout.LayoutParams.MATCH_PARENT else 0; t.weight = if (guideLarge) 0f else 1f; b.hintTexts.layoutParams = t }
        b.guide.layoutParams = lp; b.guide.requestLayout(); b.guide.invalidate()
    }

    /** Two lines: what the next photo is, and how to move there from where the phone is now. */
    private fun updateHint() {
        val lp = livePose; val t = b.guide.target
        b.next.text = if (mode == "calib") (calib ?: CalibPlanner(4000, 3000)).status().next else scanBase
        if (t == null) { b.move.visibility = View.GONE; return }
        b.move.visibility = View.VISIBLE
        if (lp == null) { b.move.text = getString(R.string.hint_no_board); b.move.setTextColor(col(R.color.warn)); return }
        val daz = ((t.az - lp.az + 540) % 360) - 180
        val del = t.el - lp.el
        val dd = t.dist - lp.dist * 1000
        val rollNow = abs(((lp.roll + 90) % 180 + 180) % 180 - 90)          // 0 upright .. 90 sideways
        val wantRoll = abs(t.roll) > 45
        val parts = ArrayList<String>()
        val azOk = azKnown || demoPose != null
        if (azOk && abs(daz) > 15 && t.el < 80) parts += (if (daz > 0) getString(R.string.move_left) else getString(R.string.move_right)) + " " + fa(abs(daz).toInt()) + "°"
        if (abs(del) > 8) parts += if (del > 0) getString(R.string.move_up) else getString(R.string.move_down)
        if (abs(dd) > 70) parts += if (dd > 0) getString(R.string.move_back) else getString(R.string.move_closer)
        if (wantRoll && rollNow < 45) parts += getString(R.string.move_roll)
        if (!wantRoll && rollNow > 45 && mode == "calib") parts += getString(R.string.move_unroll)
        if (!azOk) parts += getString(R.string.hint_need_board_once)
        if (!seenBoard && parts.isEmpty()) { b.move.text = getString(R.string.hint_no_board); b.move.setTextColor(col(R.color.warn)); return }
        b.move.text = if (parts.isEmpty()) getString(R.string.hint_ok) else parts.joinToString(" · ")
        b.move.setTextColor(col(if (parts.isEmpty()) R.color.ok else R.color.warn))
    }

    // what just happened, over the bottom of the picture; fades away by itself
    private val hideStatus = Runnable { b.status.animate().alpha(0f).setDuration(250).withEndAction { b.status.visibility = View.INVISIBLE }.start() }
    private fun showStatus(text: String, colorRes: Int = 0, sticky: Boolean = false) {
        b.status.removeCallbacks(hideStatus); b.status.animate().cancel()
        b.status.text = text; b.status.setTextColor(if (colorRes == 0) 0xFFFFFFFF.toInt() else col(colorRes))
        b.status.alpha = 1f; b.status.visibility = View.VISIBLE
        if (!sticky) b.status.postDelayed(hideStatus, if (colorRes == 0 || colorRes == R.color.ok) 2600L else 5200L)
    }

    private fun toast(t: String) = Toast.makeText(this, t, Toast.LENGTH_SHORT).show()

    companion object {
        const val BLUR_BUDGET_PX = 0.8          // motion smear a photo may have (full-resolution pixels)
        const val STEADY_MS = 120L              // the phone must have been this still for this long before the shutter fires
        const val STEADY_TIMEOUT_MS = 1500L     // ...but the shutter never waits longer than this
        const val AUTO_STEADY_MS = 350L         // automatic shutter: a little more patience
    }
}
