package ir.scan3d.app

import android.app.Activity
import android.content.Intent
import android.os.Bundle
import android.view.View
import android.widget.ArrayAdapter
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import ir.scan3d.app.databinding.ActivitySessionsBinding
import org.json.JSONObject
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/** Past scans and calibrations kept on the phone: resume, open the result, or delete. */
class SessionsActivity : AppCompatActivity() {
    private lateinit var b: ActivitySessionsBinding
    private lateinit var store: Store
    private var rows: List<Pair<String, JSONObject>> = emptyList()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Ui.edgeToEdge(this)
        b = ActivitySessionsBinding.inflate(layoutInflater); setContentView(b.root)
        Ui.padSystemBars(b.content)
        store = Store(this)
        b.list.setOnItemClickListener { _, _, i, _ -> choose(rows[i].first, rows[i].second) }
        b.btnBack.setOnClickListener { finish() }
    }

    override fun onResume() { super.onResume(); reload() }

    private fun reload() {
        rows = store.list()
        val fmt = SimpleDateFormat("yyyy/MM/dd  HH:mm", Locale.US)
        val labels = rows.map { (sid, j) ->
            val calib = j.optBoolean("calib", false)
            val n = j.optJSONArray("shots")?.length() ?: 0
            val res = File(store.sessionDir(sid), "result.json").takeIf { it.exists() }?.let { JSONObject(it.readText()).optString("state") }
            val state = when (res) { "done" -> getString(R.string.s_done); "processing" -> getString(R.string.s_processing); "failed" -> getString(R.string.s_failed); else -> getString(R.string.s_open) }
            (if (calib) getString(R.string.s_calib) else getString(R.string.s_scan)) + " · " + getString(R.string.s_shots, Ui.fa(n)) + " · " + state + "\n" + fmt.format(Date(j.optLong("created")))
        }
        b.list.adapter = ArrayAdapter(this, R.layout.row_session, R.id.text, labels)
        b.empty.visibility = if (rows.isEmpty()) View.VISIBLE else View.GONE
    }

    private fun choose(sid: String, j: JSONObject) {
        val calib = j.optBoolean("calib", false)
        val items = arrayOf(getString(R.string.s_resume), getString(R.string.s_result), getString(R.string.s_delete))
        AlertDialog.Builder(this).setTitle(sid).setItems(items) { _, which ->
            when (which) {
                0 -> { setResult(Activity.RESULT_OK, Intent().putExtra("sid", sid)); finish() }
                1 -> startActivity(Intent(this, ResultActivity::class.java).putExtra("sid", sid).putExtra("calib", calib))
                2 -> AlertDialog.Builder(this).setMessage(getString(R.string.s_delete_q)).setPositiveButton(getString(R.string.s_delete)) { _, _ -> store.delete(sid); reload() }.setNegativeButton(android.R.string.cancel, null).show()
            }
        }.show()
    }
}
