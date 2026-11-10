package ir.scan3d.app

import android.app.Activity
import android.content.Intent
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.os.Bundle
import android.view.View
import android.view.ViewGroup
import android.widget.BaseAdapter
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import ir.scan3d.app.databinding.ActivitySessionsBinding
import ir.scan3d.app.databinding.RowSessionBinding
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.json.JSONObject
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/** Past scans and calibrations kept on the phone: resume, open the result, or delete. */
class SessionsActivity : AppCompatActivity() {
    private lateinit var b: ActivitySessionsBinding
    private lateinit var store: Store

    /** One line of the list, read from the session's files off the main thread. */
    private class Row(val sid: String, val calib: Boolean, val shots: Int, val created: Long, val state: String?, val dims: String?, val thumb: Bitmap?)
    private var rows: List<Row> = emptyList()

    private val adapter = object : BaseAdapter() {
        override fun getCount() = rows.size
        override fun getItem(i: Int) = rows[i]
        override fun getItemId(i: Int) = i.toLong()
        override fun getView(i: Int, convert: View?, parent: ViewGroup): View {
            val v = if (convert != null) RowSessionBinding.bind(convert) else RowSessionBinding.inflate(layoutInflater, parent, false)
            val r = rows[i]
            v.thumb.setImageBitmap(r.thumb)
            v.text.text = getString(if (r.calib) R.string.s_calib else R.string.s_scan) + " · " + getString(R.string.s_shots, Ui.fa(r.shots))
            v.sub.text = r.dims ?: ""; v.sub.visibility = if (r.dims == null) View.GONE else View.VISIBLE
            v.date.text = Ui.fa(SimpleDateFormat("yyyy/MM/dd  HH:mm", Locale.US).format(Date(r.created)))
            val (label, color) = when (r.state) {
                "done" -> (if (r.calib) R.string.s_done_calib else R.string.s_done) to R.color.ok
                "processing" -> R.string.s_processing to R.color.accent
                "failed" -> R.string.s_failed to R.color.bad
                else -> R.string.s_open to R.color.text_dim
            }
            v.state.text = getString(label); v.state.setTextColor(ContextCompat.getColor(this@SessionsActivity, color))
            return v.root
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Ui.edgeToEdge(this)
        b = ActivitySessionsBinding.inflate(layoutInflater); setContentView(b.root)
        Ui.padSystemBars(b.content)
        store = Store(this)
        b.list.adapter = adapter
        b.list.setOnItemClickListener { _, _, i, _ -> choose(rows[i]) }
        b.btnBack.setOnClickListener { finish() }
    }

    override fun onResume() { super.onResume(); reload() }

    private fun reload() {
        lifecycleScope.launch {
            val list = withContext(Dispatchers.IO) {
                store.list().map { (sid, j) ->
                    val dir = store.sessionDir(sid)
                    val res = try { File(dir, "result.json").takeIf { it.exists() }?.let { JSONObject(it.readText()) } } catch (_: Exception) { null }
                    var state = res?.optString("state")
                    if (state == "processing" && BuildService.runningSid != sid) state = "failed"          // a build that died
                    val fit = res?.optJSONObject("fit")
                    val dims = if (state == "done" && fit != null)
                        getString(R.string.s_dims, Ui.num(maxOf(fit.optDouble("w"), fit.optDouble("l"))), Ui.num(minOf(fit.optDouble("w"), fit.optDouble("l"))), Ui.num(fit.optDouble("h")))
                    else null
                    // the first photo of the session stands for it
                    val thumb = dir.listFiles()?.filter { it.name.startsWith("thumb") && it.name.endsWith(".jpg") }
                        ?.minByOrNull { it.name.removePrefix("thumb").removeSuffix(".jpg").toIntOrNull() ?: Int.MAX_VALUE }
                        ?.let { f -> try { BitmapFactory.decodeFile(f.absolutePath) } catch (_: Throwable) { null } }
                    Row(sid, j.optBoolean("calib", false), j.optJSONArray("shots")?.length() ?: 0, j.optLong("created"), state, dims, thumb)
                }
            }
            rows = list; adapter.notifyDataSetChanged()
            b.empty.visibility = if (rows.isEmpty()) View.VISIBLE else View.GONE
        }
    }

    private fun choose(r: Row) {
        val items = if (r.state != null) arrayOf(getString(R.string.s_result), getString(R.string.s_resume), getString(R.string.s_delete))
                    else arrayOf(getString(R.string.s_resume), getString(R.string.s_delete))
        AlertDialog.Builder(this).setItems(items) { _, which ->
            when (items[which]) {
                getString(R.string.s_resume) -> { setResult(Activity.RESULT_OK, Intent().putExtra("sid", r.sid)); finish() }
                getString(R.string.s_result) -> startActivity(Intent(this, ResultActivity::class.java).putExtra("sid", r.sid).putExtra("calib", r.calib))
                else -> AlertDialog.Builder(this).setMessage(getString(R.string.s_delete_q))
                    .setPositiveButton(getString(R.string.s_delete)) { _, _ -> store.delete(r.sid); reload() }.setNegativeButton(getString(R.string.no), null).show()
            }
        }.show()
    }
}
