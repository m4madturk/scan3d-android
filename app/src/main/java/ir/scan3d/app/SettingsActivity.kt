package ir.scan3d.app

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.lifecycleScope
import ir.scan3d.app.databinding.ActivitySettingsBinding
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

class SettingsActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val b = ActivitySettingsBinding.inflate(layoutInflater); setContentView(b.root)
        val api = Api(this)
        b.server.setText(api.baseUrl); b.user.setText(api.user); b.password.setText(api.password)
        b.btnTest.setOnClickListener {
            api.save(b.server.text.toString(), b.user.text.toString(), b.password.text.toString())
            b.status.text = "…"
            lifecycleScope.launch(Dispatchers.IO) {
                val msg = try { api.ping(); getString(R.string.server_ok) } catch (e: Exception) { if (e.message == "401") getString(R.string.server_auth) else getString(R.string.server_fail, e.message ?: "") }
                withContext(Dispatchers.Main) { b.status.text = msg }
            }
        }
        b.btnSave.setOnClickListener { api.save(b.server.text.toString(), b.user.text.toString(), b.password.text.toString()); finish() }
    }
}
