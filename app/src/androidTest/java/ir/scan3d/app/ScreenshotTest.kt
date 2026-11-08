package ir.scan3d.app

import android.Manifest
import android.content.Intent
import android.graphics.Bitmap
import android.view.View
import androidx.test.core.app.ActivityScenario
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.rule.GrantPermissionRule
import ir.scan3d.app.engine.Mesh
import org.json.JSONArray
import org.json.JSONObject
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import java.io.File

/**
 * Drives every screen with synthetic data and saves a PNG of each into filesDir/screens.
 * CI pulls those files and commits them to the `screens` branch so the layout can be reviewed.
 */
@RunWith(AndroidJUnit4::class)
class ScreenshotTest {
    @get:Rule
    val perms: GrantPermissionRule = GrantPermissionRule.grant(Manifest.permission.CAMERA)

    private val ctx get() = ApplicationProvider.getApplicationContext<android.content.Context>()
    private val tag = InstrumentationRegistry.getArguments().getString("screenTag") ?: "default"
    private val outDir get() = File(ctx.filesDir, "screens").apply { mkdirs() }

    private fun shoot(name: String) {
        Thread.sleep(600)   // let the frame settle
        val bmp = InstrumentationRegistry.getInstrumentation().uiAutomation.takeScreenshot() ?: error("no screenshot")
        File(outDir, "${tag}_$name.png").outputStream().use { bmp.compress(Bitmap.CompressFormat.PNG, 90, it) }
        bmp.recycle()
    }

    private fun boxMesh(sx: Float, sy: Float, sz: Float): Mesh {
        val v = floatArrayOf(0f, 0f, 0f, sx, 0f, 0f, sx, sy, 0f, 0f, sy, 0f, 0f, 0f, sz, sx, 0f, sz, sx, sy, sz, 0f, sy, sz)
        val t = intArrayOf(0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7)
        return Mesh(v, t)
    }

    private fun scanSession(sid: String, calib: Boolean, nShots: Int): File {
        val st = Store(ctx)
        val s = Session(ctx, sid, if (calib) "calibration" else "scan", if (calib) "free" else "18", calib)
        for (i in 1..nShots) {
            val sh = Shot(i, "k$i"); sh.state = "done"; sh.w = 4000; sh.h = 3000; sh.orientation = 6; sh.landscape = i % 5 == 0
            sh.pose = Pose((i * 60.0) % 360, 20.0 + (i % 3) * 25, 0.4, DoubleArray(3), DoubleArray(3))
            sh.slotKey = s.slotOf(sh.pose)
            sh.stickers = (0 until 4).map { Sticker(80 + it, FloatArray(8) { k -> 1000f + 50 * k }) }
            s.shots.add(sh)
        }
        st.save(s)
        return st.sessionDir(sid)
    }

    private fun noTutorial() { ctx.getSharedPreferences("scan3d", android.content.Context.MODE_PRIVATE).edit().putBoolean("tutorial_seen", true).commit() }

    @Test fun mainScan() {
        noTutorial(); scanSession("s_demo", false, 7)
        // pretend the board is seen from 5 o'clock, 30 degrees up, 36 cm, upright
        ActivityScenario.launch<MainActivity>(Intent(ctx, MainActivity::class.java).putExtra("sid", "s_demo").putExtra("demoPose", doubleArrayOf(150.0, 30.0, 0.36, 0.0))).use {
            Thread.sleep(3500)   // camera start
            shoot("01_main_scan")
            it.onActivity { a -> a.findViewById<View>(R.id.guide).performClick() }
            Thread.sleep(800)
            shoot("01b_main_scan_bigguide")
        }
    }

    @Test fun mainCalib() {
        noTutorial()
        ActivityScenario.launch<MainActivity>(Intent(ctx, MainActivity::class.java).putExtra("mode", "calib").putExtra("demoPose", doubleArrayOf(180.0, 80.0, 0.30, 0.0))).use {
            Thread.sleep(3000)
            shoot("02_main_calib")
        }
    }

    @Test fun tutorial() {
        for ((i, pg) in listOf(0, 2, 4, 6).withIndex()) {
            ActivityScenario.launch<TutorialActivity>(Intent(ctx, TutorialActivity::class.java).putExtra("page", pg)).use {
                Thread.sleep(900)
                shoot("07_tutorial_p$pg")
            }
        }
    }

    @Test fun resultProcessing() {
        val dir = scanSession("s_proc", false, 12)
        File(dir, "result.json").writeText(JSONObject().put("state", "processing").put("progress", 0.63).put("message", "تراش نهایی (0.41 mm)…").toString())
        File(dir, "log.txt").writeText("using camera profile as prior\nframework: 12 photos\nBA: sigma0 0.31 px\n")
        ActivityScenario.launch<ResultActivity>(Intent(ctx, ResultActivity::class.java).putExtra("sid", "s_proc")).use {
            Thread.sleep(1500)
            shoot("03_result_processing")
        }
    }

    @Test fun resultDone() {
        val dir = scanSession("s_done", false, 14)
        // a tilted rounded box as the "scan", run through the real fitter so the clean section shows real numbers
        val truth = ir.scan3d.app.engine.ShapeFit.Params(0.0, 0.0, Math.toRadians(12.0), 66.0, 58.0, 27.0, 10.0, 5.0, Math.toRadians(2.5), Math.toRadians(-1.0))
        val mesh = ir.scan3d.app.engine.ShapeFit.mesh(truth.copy().also { it.tx = 0.0; it.ty = 0.0 })
        mesh.writeStl(File(dir, "model.stl"))
        Preview.render(mesh, File(dir, "preview.png"))
        val fit = ir.scan3d.app.engine.ShapeFit.fit(mesh)!!
        val clean = ir.scan3d.app.engine.ShapeFit.mesh(fit.params)
        clean.writeStl(File(dir, "model_clean.stl")); Preview.render(clean, File(dir, "preview_clean.png"))
        val rep = Pipeline.Report(doubleArrayOf(66.4, 58.3, 27.0), 97.2, 14, 7, 0.31, 7, 14,
            listOf("only 2 rolled photos; focal length and principal point weakly separated"), 148.0, mesh.triangleCount, fit)
        val j = Pipeline.reportJson(rep).put("state", "done").put("message", "14 views, 148 s")
        j.getJSONObject("fit").put("tilt_deg", 2.7)        // as a bent sheet would show
        File(dir, "result.json").writeText(j.toString())
        ActivityScenario.launch<ResultActivity>(Intent(ctx, ResultActivity::class.java).putExtra("sid", "s_done")).use {
            Thread.sleep(1500)
            shoot("04_result_done")
        }
    }

    @Test fun resultCalib() {
        val dir = scanSession("c_demo", true, 26)
        File(dir, "result.json").writeText(JSONObject().put("state", "done").put("f_px", 3012.4).put("sigma_f", 0.8).put("cx", 2003.1).put("cy", 1497.6)
            .put("k1", 0.0612).put("rms_px", 0.42).put("frames", 26).put("warnings", JSONArray()).put("message", "profile saved (k1k2); f ± 0.8 px").toString())
        ActivityScenario.launch<ResultActivity>(Intent(ctx, ResultActivity::class.java).putExtra("sid", "c_demo").putExtra("calib", true)).use {
            Thread.sleep(1200)
            shoot("05_result_calib")
        }
    }

    @Test fun sessions() {
        scanSession("s_a", false, 18); scanSession("c_b", true, 25); scanSession("s_c", false, 3)
        File(Store(ctx).sessionDir("s_a"), "result.json").writeText("""{"state":"done"}""")
        ActivityScenario.launch<SessionsActivity>(Intent(ctx, SessionsActivity::class.java)).use {
            Thread.sleep(1200)
            shoot("06_sessions")
        }
    }
}
