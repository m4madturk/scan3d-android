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

/** Polls the server for the build (or calibration) result and shows it. */
class ResultActivity : AppCompatActivity() {
    private lateinit var b: ActivityResultBinding
    private lateinit var api: Api

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        b = ActivityResultBinding.inflate(layoutInflater); setContentView(b.root)
        api = Api(this)
        val sid = intent.getStringExtra("sid") ?: run { finish(); return }
        val calib = intent.getBooleanExtra("calib", false)
        val col = if (calib) "calib" else "sessions"
        b.title.text = if (calib) getString(R.string.calib_result) else getString(R.string.result)
        lifecycleScope.launch(Dispatchers.IO) {
            var last = ""
            while (isActive) {
                try {
                    val d = api.getDoc("$col/$sid/status/result")
                    val s = d?.toString() ?: ""
                    if (s != last) { last = s; withContext(Dispatchers.Main) { show(d, sid, col) } }
                    if (d != null && d.optString("state") in listOf("done", "failed")) break
                } catch (e: Exception) { withContext(Dispatchers.Main) { b.state.text = "…" + (e.message ?: "") } }
                delay(2500)
            }
        }
    }

    private fun show(d: JSONObject?, sid: String, col: String) {
        if (d == null) { b.state.text = getString(R.string.waiting); return }
        val st = d.optString("state")
        b.state.text = when (st) { "processing" -> getString(R.string.processing); "done" -> getString(R.string.done); "failed" -> getString(R.string.failed) + " " + d.optString("message"); else -> st }
        val sb = StringBuilder()
        d.optJSONArray("size_mm")?.let { a -> sb.append("size: ").append((0 until a.length()).joinToString(" x ") { "%.1f".format(a.getDouble(it)) }).append(" mm\n") }
        if (d.has("volume_cm3")) sb.append("volume: ${d.optDouble("volume_cm3")} cm3\n")
        if (d.has("views_used")) sb.append("views: ${d.optInt("views_used")}  marker planes: ${d.optInt("marker_planes")}\n")
        if (d.has("f_px")) sb.append("f = %.1f px   c = (%.1f, %.1f)\n".format(d.optDouble("f_px"), d.optDouble("cx"), d.optDouble("cy")))
        if (d.has("rms_px")) sb.append("RMS %.3f px   frames %d\n".format(d.optDouble("rms_px"), d.optInt("frames")))
        if (d.has("message") && st != "failed") sb.append(d.optString("message")).append("\n")
        d.optJSONArray("warnings")?.let { w -> for (i in 0 until w.length()) sb.append("! ").append(w.getString(i)).append("\n") }
        b.info.text = sb.toString()
        val prev = d.optString("preview_url", "")
        if (prev.isNotEmpty()) lifecycleScope.launch(Dispatchers.IO) {
            try { val bytes = api.getBytes(prev); val bmp = BitmapFactory.decodeByteArray(bytes, 0, bytes.size); withContext(Dispatchers.Main) { b.preview.setImageBitmap(bmp) } } catch (_: Exception) {}
        }
        val stl = d.optString("stl_url", "")
        b.btnShare.isEnabled = stl.isNotEmpty()
        b.btnShare.setOnClickListener {
            lifecycleScope.launch(Dispatchers.IO) {
                try {
                    val f = File(getExternalFilesDir(null), "model_$sid.stl"); f.writeBytes(api.getBytes(stl))
                    val uri = FileProvider.getUriForFile(this@ResultActivity, "$packageName.files", f)
                    withContext(Dispatchers.Main) {
                        startActivity(Intent.createChooser(Intent(Intent.ACTION_SEND).setType("model/stl").putExtra(Intent.EXTRA_STREAM, uri).addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION), "STL"))
                    }
                } catch (e: Exception) { withContext(Dispatchers.Main) { b.state.text = "download: ${e.message}" } }
            }
        }
    }
}
