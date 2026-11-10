package ir.scan3d.app

import android.content.Intent
import android.graphics.BitmapFactory
import android.os.Bundle
import android.view.View
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.core.content.FileProvider
import androidx.lifecycle.lifecycleScope
import ir.scan3d.app.databinding.ActivityResultBinding
import ir.scan3d.app.engine.Stl
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.json.JSONObject
import java.io.File
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

/**
 * Progress of the on-phone build, then the result: the model to turn around, its dimensions with their
 * repeatability, the things that limit the accuracy of THIS scan (in plain words), and the STL files.
 * The raw numbers and the processing log stay available under "technical details".
 */
class ResultActivity : AppCompatActivity() {
    private lateinit var b: ActivityResultBinding
    private lateinit var dir: File
    private var sid = ""
    private var calib = false
    private var cleanModel: Stl.Model? = null
    private var rawModel: Stl.Model? = null
    private var showClean = true
    private var glFailed = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Ui.edgeToEdge(this)
        b = ActivityResultBinding.inflate(layoutInflater); setContentView(b.root)
        Ui.padSystemBars(b.content)
        sid = intent.getStringExtra("sid") ?: run { finish(); return }
        calib = intent.getBooleanExtra("calib", false)
        b.title.text = getString(if (calib) R.string.calib_result else R.string.result)
        b.btnBack.setOnClickListener { finish() }
        dir = Store(this).sessionDir(sid)
        val resultFile = File(dir, "result.json")
        b.btnDetails.setOnClickListener {
            val open = b.details.visibility != View.VISIBLE
            b.details.visibility = if (open) View.VISIBLE else View.GONE
            if (open) { loadLog(); b.scroll.post { b.scroll.smoothScrollTo(0, b.detailsCard.top) } }
        }
        b.btnExport.setOnClickListener { export() }
        b.tabClean.setOnClickListener { showClean = true; showModel() }
        b.tabRaw.setOnClickListener { showClean = false; showModel() }
        b.model.onFailure = { glFailed = true; showModel() }
        lifecycleScope.launch(Dispatchers.IO) {
            var last = "\u0000"
            while (isActive) {
                val txt = try { if (resultFile.exists()) resultFile.readText() else "" } catch (_: Exception) { "" }
                val d = try { if (txt.isEmpty()) null else JSONObject(txt) } catch (_: Exception) { null }
                // a file that still says "processing" while no build runs in this process is a build that died
                val dead = d?.optString("state") == "processing" && BuildService.runningSid != sid
                if (txt != last || dead) { last = txt; withContext(Dispatchers.Main) { show(if (dead) JSONObject().put("state", "failed").put("message", "interrupted") else d) } }
                val st = d?.optString("state")
                if (st == "done" || st == "failed" || dead) break
                delay(600)
            }
        }
    }

    override fun onResume() { super.onResume(); b.model.onResume() }
    override fun onPause() { b.model.onPause(); super.onPause() }

    private fun vis(v: View, on: Boolean) { v.visibility = if (on) View.VISIBLE else View.GONE }
    private fun col(id: Int) = ContextCompat.getColor(this, id)

    private fun show(d: JSONObject?) {
        if (isDestroyed) return
        val st = d?.optString("state") ?: ""
        val done = st == "done"
        vis(b.stateCard, !done)
        if (!done) {
            vis(b.progress, st != "failed"); vis(b.hint, st != "failed")
            b.state.setTextColor(col(if (st == "failed") R.color.bad else R.color.text))
            when (st) {
                "processing" -> {
                    b.state.text = getString(if (calib) R.string.processing_calib else R.string.processing)
                    b.stateMsg.text = d!!.optString("message")
                    val p = (d.optDouble("progress", 0.0) * 100).toInt()
                    b.progress.isIndeterminate = p <= 0; b.progress.progress = p
                }
                "failed" -> { b.state.text = getString(R.string.failed); b.stateMsg.text = failure(d!!.optString("message")) }
                else -> { b.state.text = getString(R.string.waiting); b.stateMsg.text = ""; b.progress.isIndeterminate = true }
            }
            vis(b.viewerBox, false); vis(b.dimsCard, false); vis(b.calibCard, false); vis(b.warnCard, false); vis(b.btnShare, false); vis(b.btnShareClean, false)
            if (b.details.visibility == View.VISIBLE) loadLog()
            return
        }
        d!!
        if (calib) showCalib(d) else showScan(d)
        if (b.details.visibility == View.VISIBLE) loadLog()
    }

    /** A failure in words the user can act on; anything unknown is shown as it is. */
    private fun failure(msg: String): String = when {
        msg == "interrupted" -> getString(R.string.f_interrupted)
        msg.contains("fewer than 2 shots") -> getString(R.string.f_board)
        msg.contains("object not found") -> getString(R.string.f_object)
        msg.contains("usable silhouettes") -> getString(R.string.f_sil)
        msg.contains("session not found") -> getString(R.string.f_session)
        else -> msg
    }

    // ---------------- scan ----------------
    private fun showScan(d: JSONObject) {
        val fit = d.optJSONObject("fit")
        val rawStl = File(dir, "model.stl"); val cleanStl = File(dir, "model_clean.stl")
        val hasClean = fit != null && cleanStl.exists()
        // dimensions
        vis(b.dimsCard, true)
        if (hasClean) {
            fit!!
            val w = fit.optDouble("w"); val l = fit.optDouble("l"); val h = fit.optDouble("h")
            val sw = fit.optDouble("sigma_w", Double.NaN); val sl = fit.optDouble("sigma_l", Double.NaN); val sh = fit.optDouble("sigma_h", Double.NaN)
            b.dimsTitle.text = getString(R.string.dims_title)
            b.dimA.text = Ui.num(max(w, l)); b.dimB.text = Ui.num(min(w, l)); b.dimC.text = Ui.num(h)
            // repeatability at 95 %: twice the leave-one-photo-out sigma
            pm(b.dimAs, if (w >= l) sw else sl); pm(b.dimBs, if (w >= l) sl else sw); pm(b.dimCs, sh)
            val within = fit.optDouble("within", 0.0)
            vis(b.grade, true)
            b.grade.text = getString(if (within >= 0.75) R.string.grade_a else if (within >= 0.6) R.string.grade_b else R.string.grade_c)
            b.grade.setTextColor(col(if (within >= 0.6) R.color.ok else R.color.warn))
            b.dimsNote.text = getString(R.string.dims_note, Ui.num(fit.optDouble("rc")), Ui.num(fit.optDouble("rz")), Ui.num(fit.optDouble("rb")),
                Ui.num(100 * within, 0), Ui.fa(fit.optInt("edges")), Ui.num(fit.optDouble("median_mm"), 2)) +
                (if (!sw.isNaN()) "\n" + getString(R.string.dims_pm_note) else "")
        } else {
            val a = d.optJSONArray("size_mm")
            val x = a?.optDouble(0) ?: 0.0; val y = a?.optDouble(1) ?: 0.0; val z = a?.optDouble(2) ?: 0.0
            b.dimsTitle.text = getString(R.string.dims_title_raw)
            b.dimA.text = Ui.num(max(x, y)); b.dimB.text = Ui.num(min(x, y)); b.dimC.text = Ui.num(z)
            vis(b.dimAs, false); vis(b.dimBs, false); vis(b.dimCs, false); vis(b.grade, false)
            b.dimsNote.text = getString(R.string.clean_none) + "\n" + getString(R.string.dims_note_raw)
        }
        // what limits the accuracy of this scan
        val warn = ArrayList<String>()
        if (hasClean) {
            fit!!
            val tilt = fit.optDouble("tilt_deg", 0.0)
            if (tilt > 2.0) warn += getString(R.string.w_tilt, Ui.num(tilt), Ui.num(0.5 * max(fit.optDouble("w"), fit.optDouble("l")) * sin(Math.toRadians(tilt))))
            if (fit.optDouble("within", 1.0) < 0.6) warn += getString(R.string.w_loose, Ui.num(100 * fit.optDouble("within"), 0))
        }
        val bend = d.optDouble("sheet_bend_mm", 0.0)
        if (bend > 1.0) warn += getString(R.string.w_bend, Ui.num(bend))
        val s0 = d.optDouble("sigma0_px", 0.0)
        if (s0 > 0.6) warn += getString(R.string.w_sigma, Ui.num(s0, 2))
        val views = d.optInt("views_used", 0)
        if (views in 1..9) warn += getString(R.string.w_views, Ui.fa(views))
        d.optJSONArray("warnings")?.let { w -> for (i in 0 until w.length()) engineWarning(w.optString(i))?.let { warn += it } }
        showWarnings(warn)
        // files
        vis(b.btnShareClean, hasClean); vis(b.btnShare, rawStl.exists())
        b.btnShareClean.setOnClickListener { shareStl(cleanStl, "scan_${sid}_clean.stl") }
        b.btnShare.setOnClickListener { shareStl(rawStl, "scan_$sid.stl") }
        // technical details
        val info = StringBuilder()
        info.append(getString(R.string.d_views, Ui.fa(views))).append('\n')
        info.append(getString(R.string.d_stickers, Ui.fa(d.optInt("stickers")))).append('\n')
        info.append(getString(R.string.d_sigma0, Ui.num(s0, 2))).append('\n')
        if (d.optDouble("focal_px", 0.0) > 0) info.append(getString(R.string.d_focal, Ui.num(d.optDouble("focal_px"), 0))).append('\n')
        info.append(getString(R.string.d_bend, Ui.num(bend))).append('\n')
        fit?.let { f ->
            info.append(getString(R.string.d_tilt, Ui.num(f.optDouble("tilt_deg")))).append('\n')
            info.append(getString(R.string.d_edges, Ui.fa(f.optInt("edges")), Ui.fa(f.optInt("views")))).append('\n')
        }
        d.optJSONArray("size_mm")?.let { a -> info.append(getString(R.string.d_raw, Ui.num(a.optDouble(0)), Ui.num(a.optDouble(1)), Ui.num(a.optDouble(2)), Ui.num(d.optDouble("volume_cm3")))).append('\n') }
        info.append(getString(R.string.d_time, Ui.num(d.optDouble("seconds"), 0), Ui.fa(d.optInt("triangles"))))
        b.info.text = info
        // the model itself
        vis(b.viewerBox, true); vis(b.tabs, hasClean)
        showClean = hasClean
        lifecycleScope.launch(Dispatchers.IO) {
            val c = if (hasClean) try { Stl.load(cleanStl) } catch (_: Throwable) { null } else null
            val r = try { Stl.load(rawStl) } catch (_: Throwable) { null }
            withContext(Dispatchers.Main) { cleanModel = c; rawModel = r; showModel() }
        }
    }

    private fun pm(v: android.widget.TextView, sigma: Double) {
        vis(v, !sigma.isNaN())
        if (!sigma.isNaN()) v.text = getString(R.string.pm, Ui.num(max(0.01, 2 * sigma), 2))
    }

    private fun showModel() {
        if (isDestroyed) return
        b.tabClean.isSelected = showClean; b.tabRaw.isSelected = !showClean
        val m = if (showClean) cleanModel else rawModel
        val png = File(dir, if (showClean) "preview_clean.png" else "preview.png")
        if (m == null || glFailed) {            // no OpenGL or an unreadable file: the rendered picture instead
            vis(b.model, false); vis(b.viewerHint, false)
            if (png.exists()) { b.preview.setImageBitmap(BitmapFactory.decodeFile(png.absolutePath)); vis(b.preview, true) } else vis(b.preview, false)
            return
        }
        vis(b.preview, false); vis(b.model, true); vis(b.viewerHint, true)
        if (showClean) b.model.show(m, 0.36f, 0.78f, 0.74f) else b.model.show(m, 0.62f, 0.68f, 0.72f)
    }

    private fun showWarnings(list: List<String>) {
        vis(b.warnCard, list.isNotEmpty())
        b.warnings.text = list.joinToString("\n") { "• $it" }
    }

    /** The solver's own (English) warnings, in Persian. Unknown ones are passed through; duplicates of ours dropped. */
    private fun engineWarning(w: String): String? {
        Regex("""intrinsics (\S+) correlation""").find(w)?.let { return getString(R.string.w_corr, it.groupValues[1]) }
        Regex("""only (\d+) framework photos""").find(w)?.let { return getString(R.string.w_framework, Ui.fa(it.groupValues[1])) }
        Regex("""only (\d+) rolled""").find(w)?.let { return getString(R.string.w_rolled, Ui.fa(it.groupValues[1])) }
        Regex("""frame coverage (\d+/\d+)""").find(w)?.let { return getString(R.string.w_coverage, Ui.fa(it.groupValues[1])) }
        if (w.startsWith("few tilted views")) return getString(R.string.w_tilts)
        Regex("""^(\d+) photo\(s\) were taken while the phone was moving""").find(w)?.let { return getString(R.string.w_blur, Ui.fa(it.groupValues[1])) }
        Regex("""RMS ([\d.]+) px is high""").find(w)?.let { return getString(R.string.w_rms, Ui.fa(it.groupValues[1]).replace('.', '٫')) }
        if (w.startsWith("object tilted") || w.startsWith("the sheet is bent")) return null          // shown from the numbers above
        return w
    }

    // ---------------- calibration ----------------
    private fun showCalib(d: JSONObject) {
        vis(b.viewerBox, false); vis(b.dimsCard, false); vis(b.btnShare, false); vis(b.btnShareClean, false)
        vis(b.calibCard, true)
        b.calibHead.text = getString(R.string.c_head_ok)
        b.calibBody.text = getString(R.string.c_body, Ui.num(d.optDouble("f_px"), 1), Ui.num(d.optDouble("sigma_f"), 1), Ui.num(d.optDouble("cx"), 1), Ui.num(d.optDouble("cy"), 1),
            Ui.num(d.optDouble("k1"), 4), Ui.num(d.optDouble("k2", 0.0), 4), Ui.num(d.optDouble("rms_px"), 2), Ui.fa(d.optInt("frames")), Ui.fa(d.optInt("coverage")), Ui.fa(d.optInt("coverage_cells", 24))) +
            "\n\n" + getString(R.string.c_use)
        val warn = ArrayList<String>()
        val bend = d.optDouble("sheet_bend_mm", 0.0)
        if (bend > 1.0) warn += getString(R.string.w_bend, Ui.num(bend))
        d.optJSONArray("warnings")?.let { w -> for (i in 0 until w.length()) engineWarning(w.optString(i))?.let { warn += it } }
        showWarnings(warn)
        b.info.text = d.optString("message")
    }

    // ---------------- files ----------------
    private fun loadLog() {
        lifecycleScope.launch(Dispatchers.IO) {
            val t = try { File(dir, "log.txt").takeIf { it.exists() }?.readText() ?: "" } catch (_: Exception) { "" }
            withContext(Dispatchers.Main) { b.log.text = t }
        }
    }

    private fun export() {
        b.btnExport.isEnabled = false
        lifecycleScope.launch(Dispatchers.IO) {
            val out = File(getExternalFilesDir(null), "scan3d_$sid.zip")
            val ok = try { Store(this@ResultActivity).exportZip(sid, out); true } catch (e: Exception) { false }
            withContext(Dispatchers.Main) {
                b.btnExport.isEnabled = true
                if (!ok) { Toast.makeText(this@ResultActivity, getString(R.string.export_failed), Toast.LENGTH_SHORT).show(); return@withContext }
                val uri = FileProvider.getUriForFile(this@ResultActivity, "$packageName.files", out)
                startActivity(Intent.createChooser(Intent(Intent.ACTION_SEND).setType("application/zip").putExtra(Intent.EXTRA_STREAM, uri).addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION), "zip"))
            }
        }
    }

    private fun shareStl(src: File, name: String) {
        lifecycleScope.launch(Dispatchers.IO) {
            val out = File(getExternalFilesDir(null), name)
            val ok = try { src.copyTo(out, overwrite = true); true } catch (_: Exception) { false }
            withContext(Dispatchers.Main) {
                if (!ok) { Toast.makeText(this@ResultActivity, getString(R.string.export_failed), Toast.LENGTH_SHORT).show(); return@withContext }
                val uri = FileProvider.getUriForFile(this@ResultActivity, "$packageName.files", out)
                startActivity(Intent.createChooser(Intent(Intent.ACTION_SEND).setType("model/stl").putExtra(Intent.EXTRA_STREAM, uri).addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION), "STL"))
            }
        }
    }
}
