package ir.scan3d.app

import android.content.Intent
import android.graphics.BitmapFactory
import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.FileProvider
import androidx.lifecycle.lifecycleScope
import ir.scan3d.app.databinding.ActivityResultBinding
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.json.JSONObject
import java.io.File

/** Shows progress of the on-phone build and the finished model (preview, numbers, STL share). */
class ResultActivity : AppCompatActivity() {
    private lateinit var b: ActivityResultBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Ui.edgeToEdge(this)
        b = ActivityResultBinding.inflate(layoutInflater); setContentView(b.root)
        Ui.padSystemBars(b.content)
        val sid = intent.getStringExtra("sid") ?: run { finish(); return }
        val calib = intent.getBooleanExtra("calib", false)
        b.title.text = if (calib) getString(R.string.calib_result) else getString(R.string.result)
        b.btnShare.visibility = if (calib) android.view.View.GONE else android.view.View.VISIBLE
        b.btnBack.setOnClickListener { finish() }
        val dir = Store(this).sessionDir(sid)
        val resultFile = File(dir, "result.json")
        b.btnLog.setOnClickListener { val f = File(dir, "log.txt"); b.log.text = if (f.exists()) f.readText() else "-"; b.log.visibility = if (b.log.visibility == android.view.View.VISIBLE) android.view.View.GONE else android.view.View.VISIBLE }
        b.btnExport.setOnClickListener {
            b.btnExport.isEnabled = false
            lifecycleScope.launch(Dispatchers.IO) {
                val out = File(getExternalFilesDir(null), "scan3d_$sid.zip")
                val ok = try { Store(this@ResultActivity).exportZip(sid, out); true } catch (e: Exception) { false }
                withContext(Dispatchers.Main) {
                    b.btnExport.isEnabled = true
                    if (!ok) { android.widget.Toast.makeText(this@ResultActivity, "export failed", android.widget.Toast.LENGTH_SHORT).show(); return@withContext }
                    val uri = FileProvider.getUriForFile(this@ResultActivity, "$packageName.files", out)
                    startActivity(Intent.createChooser(Intent(Intent.ACTION_SEND).setType("application/zip").putExtra(Intent.EXTRA_STREAM, uri).addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION), "zip"))
                }
            }
        }
        lifecycleScope.launch(Dispatchers.IO) {
            var last = ""
            while (isActive) {
                val txt = if (resultFile.exists()) resultFile.readText() else ""
                if (txt != last) { last = txt; val d = try { if (txt.isEmpty()) null else JSONObject(txt) } catch (_: Exception) { null }; withContext(Dispatchers.Main) { show(d, dir, sid) } }
                if (txt.contains("\"state\":\"done\"") || txt.contains("\"state\":\"failed\"")) break
                delay(700)
            }
        }
    }

    private fun show(d: JSONObject?, dir: File, sid: String) {
        if (d == null) { b.state.text = getString(R.string.waiting); b.progress.isIndeterminate = true; return }
        b.progress.isIndeterminate = false
        val st = d.optString("state")
        b.progress.visibility = if (st == "processing") android.view.View.VISIBLE else android.view.View.GONE
        b.hint.visibility = if (st == "processing") android.view.View.VISIBLE else android.view.View.GONE
        b.progress.progress = (d.optDouble("progress", 0.0) * 100).toInt()
        b.state.text = when (st) { "processing" -> getString(R.string.processing) + " " + d.optString("message"); "done" -> getString(R.string.done); "failed" -> getString(R.string.failed) + " " + d.optString("message"); else -> st }
        val sb = StringBuilder()
        d.optJSONArray("size_mm")?.let { a -> sb.append("size  ").append((0 until a.length()).joinToString(" x ") { "%.2f".format(a.getDouble(it)) }).append(" mm\n") }
        if (d.has("volume_cm3")) sb.append("volume  %.2f cm3\n".format(d.optDouble("volume_cm3")))
        if (d.has("views_used")) sb.append("views ${d.optInt("views_used")}   marker planes ${d.optInt("marker_planes")}   stickers ${d.optInt("stickers")}\n")
        if (d.has("sigma0_px")) sb.append("sigma0  %.3f px   framework photos %d\n".format(d.optDouble("sigma0_px"), d.optInt("framework_frames")))
        if (d.has("f_px")) sb.append("f = %.1f px  (± %.1f)   c = (%.1f, %.1f)   k1 = %.4f\n".format(d.optDouble("f_px"), d.optDouble("sigma_f"), d.optDouble("cx"), d.optDouble("cy"), d.optDouble("k1")))
        if (d.has("rms_px")) sb.append("RMS %.3f px   frames %d\n".format(d.optDouble("rms_px"), d.optInt("frames")))
        if (d.has("triangles")) sb.append("triangles ${d.optInt("triangles")}   %.0f s\n".format(d.optDouble("seconds")))
        if (d.has("message") && st == "done") sb.append(d.optString("message")).append("\n")
        d.optJSONArray("warnings")?.let { w -> for (i in 0 until w.length()) sb.append("! ").append(w.getString(i)).append("\n") }
        b.info.text = sb.toString()
        val prev = File(dir, "preview.png")
        if (st == "done" && prev.exists()) { b.preview.setImageBitmap(BitmapFactory.decodeFile(prev.absolutePath)); b.preview.visibility = android.view.View.VISIBLE }
        else b.preview.visibility = android.view.View.GONE
        val stl = File(dir, "model.stl")
        b.btnShare.isEnabled = st == "done" && stl.exists()
        b.btnShare.setOnClickListener {
            val out = File(getExternalFilesDir(null), "scan_$sid.stl"); stl.copyTo(out, overwrite = true)
            val uri = FileProvider.getUriForFile(this, "$packageName.files", out)
            startActivity(Intent.createChooser(Intent(Intent.ACTION_SEND).setType("model/stl").putExtra(Intent.EXTRA_STREAM, uri).addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION), "STL"))
        }
    }
}
