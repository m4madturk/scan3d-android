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
            sh.board = BoardObs(IntArray(40) { it }, FloatArray(80) { k -> 500f + 30 * k })
            sh.sharp = 120.0; sh.blurPx = if (i == nShots - 1) 2.7 else 0.3; sh.focusDist = 3.1f
            s.shots.add(sh)
            // a stand-in thumbnail: a chequered sheet with a light object on it
            val bm = Bitmap.createBitmap(240, 320, Bitmap.Config.ARGB_8888); val c = android.graphics.Canvas(bm); c.drawColor(0xFF6E6A62.toInt())
            val pn = android.graphics.Paint(android.graphics.Paint.ANTI_ALIAS_FLAG)
            for (y in 0 until 8) for (x in 0 until 6) { pn.color = if ((x + y) % 2 == 0) 0xFFF2F2EE.toInt() else 0xFF1E1E1E.toInt(); c.drawRect(30f + 30 * x, 40f + 30 * y, 60f + 30 * x, 70f + 30 * y, pn) }
            pn.color = 0xFFE9E4D8.toInt(); c.drawRoundRect(85f + 4 * (i % 3), 120f, 165f + 4 * (i % 3), 205f, 22f, 22f, pn)
            File(st.sessionDir(sid), "thumb$i.jpg").outputStream().use { bm.compress(Bitmap.CompressFormat.JPEG, 80, it) }
            bm.recycle()
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
            it.onActivity { a -> a.findViewById<View>(R.id.guide).performClick(); a.findViewById<View>(R.id.btnMore).performClick() }
            Thread.sleep(1200)
            shoot("01c_main_more")
        }
    }

    @Test fun mainShots() {
        noTutorial(); scanSession("s_shots", false, 9)
        ActivityScenario.launch<MainActivity>(Intent(ctx, MainActivity::class.java).putExtra("sid", "s_shots").putExtra("demoPose", doubleArrayOf(240.0, 22.0, 0.33, 0.0))).use {
            Thread.sleep(3000)
            it.onActivity { a -> a.findViewById<View>(R.id.btnShots).performClick() }
            Thread.sleep(1200)
            shoot("01d_main_shots")
        }
    }

    @Test fun mainCalib() {
        noTutorial()
        ActivityScenario.launch<MainActivity>(Intent(ctx, MainActivity::class.java).putExtra("mode", "calib").putExtra("demoPose", doubleArrayOf(180.0, 80.0, 0.30, 0.0))).use {
            Thread.sleep(6000)          // a cold emulator camera needs a while for its first frame
            shoot("02_main_calib")
        }
    }

    @Test fun tutorial() {
        for ((i, pg) in listOf(0, 2, 5, 8).withIndex()) {
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
        BuildService.runningSid = "s_proc"          // as if the build were running: a "processing" file without it counts as a dead build
        try {
            ActivityScenario.launch<ResultActivity>(Intent(ctx, ResultActivity::class.java).putExtra("sid", "s_proc")).use {
                Thread.sleep(1500)
                shoot("03_result_processing")
            }
        } finally { BuildService.runningSid = null }
    }

    @Test fun resultDone() {
        val dir = scanSession("s_done", false, 14)
        // a rounded box as the "scan" and as the clean model, with numbers like a real 14-photo scan
        val p = doubleArrayOf(0.0, 0.0, Math.toRadians(12.0), 66.4, 66.1, 24.9, 21.8, 7.5, 0.0, 0.0, 4.9)
        val mesh = ir.scan3d.app.engine.RayFit.mesh(p, 2.0)
        mesh.writeStl(File(dir, "model.stl")); Preview.render(mesh, File(dir, "preview.png"))
        mesh.writeStl(File(dir, "model_clean.stl")); Preview.render(mesh, File(dir, "preview_clean.png"))
        val sigma = DoubleArray(11); sigma[3] = 0.13; sigma[4] = 0.19; sigma[5] = 0.17; sigma[6] = 0.8
        val fit = Pipeline.Fit(p, sigma, 4.7, 0.78, 0.21, 3890, 2310, 14, 5)
        val rep = Pipeline.Report(doubleArrayOf(69.4, 69.3, 28.4), 97.2, 14, 7, 0.31, 7, 14,
            listOf("object tilted 4.7 deg on the sheet: it is not lying flat, so its height is uncertain by about 2.7 mm"), 21.0, mesh.triangleCount, fit, 2.5, 2881.0, DoubleArray(14) { 0.3 + 0.02 * it })
        val j = Pipeline.reportJson(rep).put("state", "done").put("message", "14 views, 21 s")
        File(dir, "result.json").writeText(j.toString())
        ActivityScenario.launch<ResultActivity>(Intent(ctx, ResultActivity::class.java).putExtra("sid", "s_done")).use {
            Thread.sleep(3000)          // STL load + first GL frame
            shoot("04_result_done")
            it.onActivity { a -> a.findViewById<android.widget.ScrollView>(R.id.scroll).fullScroll(View.FOCUS_DOWN) }
            Thread.sleep(700)
            shoot("04b_result_done_bottom")
            it.onActivity { a -> a.findViewById<View>(R.id.btnDetails).performClick() }
            Thread.sleep(900)
            shoot("04c_result_details")
        }
    }

    /** A scan whose shape was not a rounded box (only the raw hull), and one that failed. */
    @Test fun resultRawAndFailed() {
        val dir = scanSession("s_raw", false, 11)
        val p = doubleArrayOf(0.0, 0.0, 0.4, 52.0, 38.0, 31.0, 6.0, 3.0, 0.0, 0.0, 2.0)
        val mesh = ir.scan3d.app.engine.RayFit.mesh(p, 3.0)
        mesh.writeStl(File(dir, "model.stl")); Preview.render(mesh, File(dir, "preview.png"))
        val rep = Pipeline.Report(doubleArrayOf(53.1, 39.2, 32.0), 58.4, 11, 5, 0.74, 5, 11, listOf("only 6 framework photos; 12-30 recommended"), 34.0, mesh.triangleCount, null, 0.4, 2890.0, DoubleArray(11) { 0.5 })
        File(dir, "result.json").writeText(Pipeline.reportJson(rep).put("state", "done").put("message", "11 views, 34 s").toString())
        ActivityScenario.launch<ResultActivity>(Intent(ctx, ResultActivity::class.java).putExtra("sid", "s_raw")).use {
            Thread.sleep(3000)
            shoot("04d_result_raw_only")
        }
        val dir2 = scanSession("s_fail", false, 5)
        File(dir2, "result.json").writeText(JSONObject().put("state", "failed").put("message", "object not found inside the silhouettes (see log: silhouettes do not intersect)").toString())
        ActivityScenario.launch<ResultActivity>(Intent(ctx, ResultActivity::class.java).putExtra("sid", "s_fail")).use {
            Thread.sleep(1200)
            shoot("04e_result_failed")
        }
    }

    @Test fun resultCalib() {
        val dir = scanSession("c_demo", true, 26)
        File(dir, "result.json").writeText(JSONObject().put("state", "done").put("f_px", 3012.4).put("sigma_f", 0.8).put("cx", 2003.1).put("cy", 1497.6)
            .put("k1", 0.0612).put("k2", -0.1840).put("rms_px", 0.42).put("frames", 26).put("coverage", 22).put("coverage_cells", 24).put("sheet_bend_mm", 1.6)
            .put("warnings", JSONArray(listOf("only 3 rolled (landscape) frames; principal point weakly determined"))).put("message", "profile saved (k1k2); f ± 0.8 px").toString())
        ActivityScenario.launch<ResultActivity>(Intent(ctx, ResultActivity::class.java).putExtra("sid", "c_demo").putExtra("calib", true)).use {
            Thread.sleep(1200)
            shoot("05_result_calib")
        }
    }

    @Test fun sessions() {
        scanSession("s_a", false, 18); scanSession("c_b", true, 25); scanSession("s_c", false, 3)
        File(Store(ctx).sessionDir("s_a"), "result.json").writeText("""{"state":"done","fit":{"w":66.4,"l":66.1,"h":24.9}}""")
        File(Store(ctx).sessionDir("c_b"), "result.json").writeText("""{"state":"done"}""")
        ActivityScenario.launch<SessionsActivity>(Intent(ctx, SessionsActivity::class.java)).use {
            Thread.sleep(1200)
            shoot("06_sessions")
        }
    }
}
