package ir.scan3d.app

import android.content.Context
import android.content.SharedPreferences
import okhttp3.Credentials
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody.Companion.toRequestBody
import org.json.JSONArray
import org.json.JSONObject
import java.io.IOException
import java.net.URLEncoder
import java.util.concurrent.TimeUnit

/** Talks to server.py (the self-hosted scanner): documents, photo uploads, model files. */
class Api(ctx: Context) {
    private val prefs: SharedPreferences = ctx.getSharedPreferences("scan3d", Context.MODE_PRIVATE)
    private val client = OkHttpClient.Builder()
        .connectTimeout(20, TimeUnit.SECONDS).readTimeout(120, TimeUnit.SECONDS).writeTimeout(120, TimeUnit.SECONDS)
        .build()

    val baseUrl: String get() = prefs.getString("server", "")!!.trim().trimEnd('/')
    val user: String get() = prefs.getString("user", "")!!
    val password: String get() = prefs.getString("password", "")!!
    val configured: Boolean get() = baseUrl.isNotEmpty()

    fun save(server: String, user: String, password: String) =
        prefs.edit().putString("server", server).putString("user", user).putString("password", password).apply()

    private fun req(path: String): Request.Builder {
        val b = Request.Builder().url("$baseUrl/$path")
        if (user.isNotEmpty()) b.header("Authorization", Credentials.basic(user, password))
        return b
    }

    private fun enc(s: String) = URLEncoder.encode(s, "UTF-8")

    @Throws(IOException::class)
    fun getDoc(path: String): JSONObject? {
        client.newCall(req("api/doc?path=${enc(path)}").get().build()).execute().use { r ->
            if (!r.isSuccessful) throw IOException("GET $path: HTTP ${r.code}")
            val j = JSONObject(r.body!!.string())
            return if (j.optBoolean("exists")) j.getJSONObject("data") else null
        }
    }

    @Throws(IOException::class)
    fun putDoc(path: String, doc: JSONObject) {
        val body = doc.toString().toRequestBody("application/json".toMediaType())
        client.newCall(req("api/doc?path=${enc(path)}").put(body).build()).execute().use { r ->
            if (!r.isSuccessful) throw IOException("PUT $path: HTTP ${r.code} ${r.body?.string()?.take(200)}")
        }
    }

    @Throws(IOException::class)
    fun deleteDoc(path: String) {
        client.newCall(req("api/doc?path=${enc(path)}").delete().build()).execute().use { r ->
            if (!r.isSuccessful && r.code != 404) throw IOException("DELETE $path: HTTP ${r.code}")
        }
    }

    @Throws(IOException::class)
    fun listCollection(path: String): JSONArray {
        client.newCall(req("api/col?path=${enc(path)}").get().build()).execute().use { r ->
            if (!r.isSuccessful) throw IOException("GET collection $path: HTTP ${r.code}")
            return JSONArray(r.body!!.string())
        }
    }

    /** Uploads a JPEG; returns the asset id. */
    @Throws(IOException::class)
    fun uploadJpeg(bytes: ByteArray): String {
        val body = bytes.toRequestBody("image/jpeg".toMediaType())
        client.newCall(req("api/assets").post(body).build()).execute().use { r ->
            if (!r.isSuccessful) throw IOException("upload: HTTP ${r.code} ${r.body?.string()?.take(200)}")
            return JSONObject(r.body!!.string()).getString("id")
        }
    }

    @Throws(IOException::class)
    fun getBytes(relative: String): ByteArray {
        client.newCall(req(relative).get().build()).execute().use { r ->
            if (!r.isSuccessful) throw IOException("GET $relative: HTTP ${r.code}")
            return r.body!!.bytes()
        }
    }

    /** Quick connectivity check: fetch the page root. */
    @Throws(IOException::class)
    fun ping() {
        client.newCall(req("api/doc?path=ping/x").get().build()).execute().use { r ->
            if (r.code == 401) throw IOException("401")
            if (!r.isSuccessful) throw IOException("HTTP ${r.code}")
        }
    }
}
