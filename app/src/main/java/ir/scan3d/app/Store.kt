package ir.scan3d.app

import android.content.Context
import org.json.JSONArray
import org.json.JSONObject
import java.io.File

/** Sessions, shots and camera profiles on the phone (plain JSON files). */
class Store(ctx: Context) {
    private val root = File(ctx.filesDir, "sessions").apply { mkdirs() }
    private val profiles = File(ctx.filesDir, "profiles").apply { mkdirs() }
    private val appCtx = ctx.applicationContext

    fun sessionDir(sid: String) = File(root, sid).apply { mkdirs() }

    fun save(s: Session) {
        val j = s.sessionDoc().put("sid", s.sid).put("calib", s.calib)
        val arr = JSONArray(); s.shots.filter { it.state == "done" }.forEach { arr.put(it.toJson()) }
        j.put("shots", arr)
        val f = File(sessionDir(s.sid), "session.json"); val tmp = File(f.parentFile, "session.tmp")
        tmp.writeText(j.toString()); tmp.renameTo(f)
    }

    fun load(sid: String): Session? {
        val f = File(sessionDir(sid), "session.json"); if (!f.exists()) return null
        val j = JSONObject(f.readText())
        val s = Session(appCtx, sid, j.optString("name", "scan"), j.optString("mode", "18"), j.optBoolean("calib", false))
        val arr = j.optJSONArray("shots") ?: JSONArray()
        for (i in 0 until arr.length()) s.shots.add(Shot.fromJson(arr.getJSONObject(i), sessionDir(sid)))
        return s
    }

    fun list(): List<Pair<String, JSONObject>> = root.listFiles()?.filter { File(it, "session.json").exists() }
        ?.map { it.name to JSONObject(File(it, "session.json").readText()) }?.sortedByDescending { it.second.optLong("created") } ?: emptyList()

    fun delete(sid: String) { sessionDir(sid).deleteRecursively() }

    fun saveProfile(p: JSONObject) {
        val sz = p.getJSONArray("image_size")
        File(profiles, "${android.os.Build.MODEL.replace(Regex("[^A-Za-z0-9_.-]"), "_")}_${sz.getInt(0)}x${sz.getInt(1)}.json").writeText(p.toString())
    }

    /** Profile for this device at this sensor size (either orientation). */
    fun loadProfile(w: Int, h: Int): JSONObject? {
        val model = android.os.Build.MODEL.replace(Regex("[^A-Za-z0-9_.-]"), "_")
        for (name in listOf("${model}_${w}x$h.json", "${model}_${h}x$w.json")) {
            val f = File(profiles, name); if (!f.exists()) continue
            val p = JSONObject(f.readText()); val sz = p.getJSONArray("image_size")
            if (sz.getInt(0) == w && sz.getInt(1) == h) return p
            // stored in the other orientation: rotate the principal point (profile made with the sensor rotated 90 deg CW)
            val q = JSONObject(p.toString()); q.put("cx", p.getDouble("cy")); q.put("cy", sz.getInt(0) - 1 - p.getDouble("cx")); q.put("image_size", JSONArray(listOf(w, h)))
            return q
        }
        return null
    }

    fun profileFiles(): List<File> = profiles.listFiles()?.filter { it.name.endsWith(".json") } ?: emptyList()

    /** Debug bundle: session + result + log + lens profiles + quarter-size photos, as one zip. */
    fun exportZip(sid: String, out: File) {
        val dir = sessionDir(sid)
        java.util.zip.ZipOutputStream(out.outputStream().buffered()).use { z ->
            fun put(name: String, bytes: ByteArray) { z.putNextEntry(java.util.zip.ZipEntry(name)); z.write(bytes); z.closeEntry() }
            for (n in listOf("session.json", "result.json", "log.txt")) File(dir, n).takeIf { it.exists() }?.let { put("$sid/$n", it.readBytes()) }
            for (p in profileFiles()) put("$sid/profiles/${p.name}", p.readBytes())
            dir.listFiles()?.filter { it.name.startsWith("shot") && it.name.endsWith(".jpg") }?.sortedBy { it.name }?.forEach { f ->
                val o = android.graphics.BitmapFactory.Options().apply { inSampleSize = 4 }
                val bm = android.graphics.BitmapFactory.decodeFile(f.absolutePath, o) ?: return@forEach
                val bos = java.io.ByteArrayOutputStream(); bm.compress(android.graphics.Bitmap.CompressFormat.JPEG, 88, bos); bm.recycle()
                // keep the EXIF orientation so the receiver can tell how the phone was held
                val exif = androidx.exifinterface.media.ExifInterface(f.absolutePath).getAttributeInt(androidx.exifinterface.media.ExifInterface.TAG_ORIENTATION, 1)
                put("$sid/${f.nameWithoutExtension}_q_o$exif.jpg", bos.toByteArray())
            }
        }
    }

    fun profileSummary(): String? {
        val fs = profiles.listFiles()?.filter { it.name.endsWith(".json") } ?: return null
        if (fs.isEmpty()) return null
        val p = JSONObject(fs.maxByOrNull { it.lastModified() }!!.readText())
        return "f %.0f px · RMS %.2f px · %d frames".format(p.getDouble("f"), p.getDouble("rms_px"), p.getInt("frames"))
    }
}
