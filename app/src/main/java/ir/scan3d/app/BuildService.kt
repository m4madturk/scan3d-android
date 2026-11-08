package ir.scan3d.app

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Intent
import android.os.Build
import android.os.IBinder
import androidx.core.app.NotificationCompat
import ir.scan3d.app.engine.Calibrator
import ir.scan3d.app.engine.CameraPrior
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch
import org.json.JSONArray
import org.json.JSONObject
import java.io.File

/**
 * Runs the whole reconstruction (or a calibration) in the background so the user can leave the
 * screen. Writes result.json (+ model.stl) into the session folder; the UI polls that file.
 */
class BuildService : Service() {
    private val scope = CoroutineScope(Dispatchers.Default + Job())

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onCreate() { super.onCreate(); org.opencv.android.OpenCVLoader.initLocal() }   // board template needs OpenCV

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        val sid = intent?.getStringExtra("sid") ?: return START_NOT_STICKY
        val calib = intent.getBooleanExtra("calib", false)
        val convex = intent.getBooleanExtra("convex", true)
        startForeground(1, notif(getString(if (calib) R.string.processing_calib else R.string.processing_model), 0))
        scope.launch {
            val store = Store(this@BuildService)
            val dir = store.sessionDir(sid)
            val resultFile = File(dir, "result.json")
            val logFile = File(dir, "log.txt")
            logFile.writeText("")
            fun log(s: String) { logFile.appendText(s + "\n") }
            try {
                val session = store.load(sid) ?: throw IllegalStateException("session not found")
                val shots = session.shots.filter { it.state == "done" }
                write(resultFile, JSONObject().put("state", "processing").put("progress", 0.0).put("message", ""))
                if (calib) {
                    val fr = Pipeline.frameObs(shots)
                    val w = shots[0].w; val h = shots[0].h
                    val cal = Calibrator(Detector.SQUARE_M * 1000, Detector.SQUARES_X); cal.log = ::log
                    val rolled = BooleanArray(shots.size) { shots[it].landscape }
                    val r = cal.calibrate(fr, w, h, rolled)
                    val prof = JSONObject().put("device", Build.MODEL).put("image_size", JSONArray(listOf(w, h))).put("frame", "sensor")
                        .put("f", r.camera.f).put("cx", r.camera.cx).put("cy", r.camera.cy).put("k1", r.camera.k1).put("k2", r.camera.k2)
                        .put("sigma", JSONArray(r.sigma.toList())).put("rms_px", r.rms).put("frames", r.frames).put("rolled", r.rolled)
                        .put("coverage", JSONArray(listOf(r.coverage, r.coverageCells))).put("model", r.model).put("warnings", JSONArray(r.warnings))
                        .put("created", System.currentTimeMillis())
                    store.saveProfile(prof)
                    write(resultFile, JSONObject().put("state", "done").put("f_px", r.camera.f).put("cx", r.camera.cx).put("cy", r.camera.cy)
                        .put("k1", r.camera.k1).put("rms_px", r.rms).put("frames", r.frames).put("sigma_f", r.sigma[0])
                        .put("warnings", JSONArray(r.warnings)).put("message", "profile saved for ${Build.MODEL} (${r.model}); f ± %.1f px".format(r.sigma[0])))
                } else {
                    val prior = store.loadProfile(shots[0].w, shots[0].h)?.let { p ->
                        val sg = p.getJSONArray("sigma")
                        CameraPrior(ir.scan3d.app.engine.Camera(p.getDouble("f"), p.getDouble("cx"), p.getDouble("cy"), p.getDouble("k1"), p.getDouble("k2")),
                            DoubleArray(5) { maxOf(sg.getDouble(it), listOf(1.0, 1.5, 1.5, 2e-3, 5e-3)[it]) })   // never trust a prior below a sane floor
                    }
                    log(if (prior != null) "using camera profile as prior" else "no camera profile; self-calibrating")
                    val files = shots.map { File(dir, "shot${it.n}.jpg") }
                    val (mesh, rep) = Pipeline.build(shots, files, convex, prior, 0.4, { msg, p ->
                        write(resultFile, JSONObject().put("state", "processing").put("progress", p).put("message", msg))
                        nm().notify(1, notif(msg, (p * 100).toInt()))
                    }, ::log)
                    mesh.writeStl(File(dir, "model.stl"))
                    Preview.render(mesh, File(dir, "preview.png"))
                    val j = Pipeline.reportJson(rep).put("state", "done").put("stl", File(dir, "model.stl").absolutePath).put("preview", File(dir, "preview.png").absolutePath)
                        .put("message", "%d views, %.0f s".format(rep.views, rep.seconds))
                    write(resultFile, j)
                }
            } catch (e: Throwable) {
                log("FAILED: $e"); write(resultFile, JSONObject().put("state", "failed").put("message", e.message ?: e.toString()))
            } finally {
                stopForeground(STOP_FOREGROUND_REMOVE); stopSelf()
            }
        }
        return START_NOT_STICKY
    }

    private fun write(f: File, j: JSONObject) { val tmp = File(f.parentFile, f.name + ".tmp"); tmp.writeText(j.toString()); tmp.renameTo(f) }
    private fun nm() = getSystemService(NOTIFICATION_SERVICE) as NotificationManager

    private fun notif(text: String, pct: Int): Notification {
        if (Build.VERSION.SDK_INT >= 26) nm().createNotificationChannel(NotificationChannel("build", "Scan3D", NotificationManager.IMPORTANCE_LOW))
        return NotificationCompat.Builder(this, "build").setSmallIcon(android.R.drawable.ic_menu_camera).setContentTitle("Scan3D").setContentText(text)
            .setProgress(100, pct, pct == 0).setOngoing(true).build()
    }
}
