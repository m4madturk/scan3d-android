package ir.scan3d.app

import android.content.Context
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.hypot

/** One captured shot, as stored in the session file (same schema as the web app / Python engine). */
class Shot(val n: Int, val id: String) {
    var state = "busy"            // busy | done | error
    var sent = false
    var assetId: String? = null
    var w = 0; var h = 0
    var orientation = 1           // EXIF orientation tag of the stored JPEG (unreliable on some phones)
    var fileRot = 0               // clockwise rotation (0/90/180/270) that maps the stored pixels to the canonical sensor frame
    var devRoll = 0.0             // device roll from gravity at capture (0 upright, +90 top-left, -90 top-right)
    var sharp = 0.0
    var pose: Pose? = null
    var slotKey: String? = null
    var stickers: List<Sticker> = emptyList()
    var board: BoardObs? = null
    var millis = 0L
    var thumb: File? = null
    var landscape = false         // phone held sideways at capture time
    var exposureNs = 0L; var iso = 0; var focusDist = 0f

    fun toJson(): JSONObject {
        val targets = JSONObject()
        for (s in stickers) targets.put(s.id.toString(), JSONArray(s.corners.map { it.toDouble() }))
        val charuco = board?.let {
            JSONObject().put("ids", JSONArray(it.ids.toList())).put("pts", JSONArray(it.pts.map { v -> v.toDouble() }))
        }
        val p = pose?.let {
            JSONObject().put("az", it.az).put("el", it.el).put("dist_m", it.dist).put("roll", it.roll)
                .put("rvec", JSONArray(it.rvec.toList())).put("tvec", JSONArray(it.tvec.toList()))
        }
        val slot = slotKey?.let { k -> val (b, a) = k.split("-"); JSONObject().put("band", b.toInt()).put("az", a.toInt()).put("key", k) }
        return JSONObject()
            .put("n", n).put("t", System.currentTimeMillis()).put("src", "photo")
            .put("file", JSONObject().put("name", "shot$n.jpg"))
            .put("w", w).put("h", h).put("file_rot", fileRot).put("dev_roll", devRoll).put("landscape", landscape)
            .put("exif", JSONObject().put("orientation", orientation).put("exposure_ns", exposureNs).put("iso", iso)
                .put("focus_diopters", focusDist.toDouble()).put("landscape", landscape).put("app", "android"))
            .put("ms", millis).put("sharp", sharp)
            .put("pose", p ?: JSONObject.NULL).put("slot", slot ?: JSONObject.NULL)
            .put("targets", targets).put("charuco", charuco ?: JSONObject.NULL)
            .put("asset", assetId?.let { JSONObject().put("id", it) } ?: JSONObject.NULL)
    }

    companion object {
        fun fromJson(j: JSONObject, dir: File): Shot {
            val s = Shot(j.getInt("n"), "k" + j.getInt("n"))
            s.state = "done"; s.w = j.getInt("w"); s.h = j.getInt("h")
            val ex = j.optJSONObject("exif"); s.orientation = ex?.optInt("orientation", 1) ?: 1; s.landscape = j.optBoolean("landscape", ex?.optBoolean("landscape", false) ?: false)
            s.fileRot = j.optInt("file_rot", 0); s.devRoll = j.optDouble("dev_roll", 0.0)
            s.sharp = j.optDouble("sharp", 0.0); s.millis = j.optLong("ms", 0)
            j.optJSONObject("pose")?.let { p ->
                val rv = p.getJSONArray("rvec"); val tv = p.getJSONArray("tvec")
                s.pose = Pose(p.getDouble("az"), p.getDouble("el"), p.getDouble("dist_m"), DoubleArray(3) { rv.getDouble(it) }, DoubleArray(3) { tv.getDouble(it) }, p.optDouble("roll", 0.0))
            }
            s.slotKey = j.optJSONObject("slot")?.optString("key")
            val tg = j.getJSONObject("targets"); val st = ArrayList<Sticker>()
            for (k in tg.keys()) { val a = tg.getJSONArray(k); st.add(Sticker(k.toInt(), FloatArray(8) { a.getDouble(it).toFloat() })) }
            s.stickers = st
            j.optJSONObject("charuco")?.let { c -> val ia = c.getJSONArray("ids"); val pa = c.getJSONArray("pts")
                s.board = BoardObs(IntArray(ia.length()) { ia.getInt(it) }, FloatArray(pa.length()) { pa.getDouble(it).toFloat() }) }
            val th = File(dir, "thumb${s.n}.jpg"); if (th.exists()) s.thumb = th
            return s
        }
    }
}

/** A scan session: slots, coverage and protocol bookkeeping. Mirrors the web app's logic. */
class Session(ctx: Context, val sid: String, var name: String, var mode: String, val calib: Boolean = false) {
    val shots = ArrayList<Shot>()
    val dir: File = File(ctx.filesDir, "sessions/$sid").apply { mkdirs() }

    data class Layout(val az: Int, val bands: List<Pair<Double, Double>>, val clock: List<String>, val bandName: List<String>)

    companion object {
        /**
         * Marks shots as "rolled" (landscape) from the camera roll measured against the board: the minority
         * class whose roll differs by ~90 degrees from the majority. Independent of gravity and of how the
         * board lies on the table; what the solver needs is roll diversity relative to the scene.
         */
        fun markRolled(shots: List<Shot>) {
            val done = shots.filter { it.state == "done" && it.pose != null }
            if (done.isEmpty()) return
            // class A: cos(2 roll) > 0 (upright or upside down), class B: sideways
            val a = done.count { Math.cos(2 * Math.toRadians(it.pose!!.roll)) > 0 }
            val majorityA = a * 2 >= done.size
            for (s in done) { val inA = Math.cos(2 * Math.toRadians(s.pose!!.roll)) > 0; s.landscape = inA != majorityA }
        }
    }

    fun layout(): Layout? = when (mode) {
        "8" -> Layout(4, listOf(10.0 to 45.0, 45.0 to 90.0), listOf("۱۲", "۳", "۶", "۹"), listOf("پایین", "بالا"))
        "18" -> Layout(6, listOf(10.0 to 35.0, 35.0 to 60.0, 60.0 to 90.0), listOf("۱۲", "۲", "۴", "۶", "۸", "۱۰"), listOf("پایین", "وسط", "بالای سر"))
        else -> null
    }

    fun slotOf(p: Pose?): String? {
        val L = layout() ?: return null; p ?: return null
        var b = L.bands.indexOfFirst { p.el >= it.first && p.el < it.second }
        if (b < 0) b = if (p.el >= 90) L.bands.size - 1 else return null
        val w = 360.0 / L.az
        val a = floor((((p.az + w / 2) % 360) + 360) % 360 / w).toInt()
        return "$b-$a"
    }

    fun takenSlots(): Set<String> = shots.filter { it.state == "done" && it.slotKey != null }.map { it.slotKey!! }.toSet()

    fun nextSlot(): String? {
        val L = layout() ?: return null
        val taken = takenSlots()
        for (b in L.bands.indices) for (a in 0 until L.az) { val k = "$b-$a"; if (k !in taken) return k }
        return null
    }

    data class Coverage(val id: Int, val n: Int, val ready: Boolean, val az: Double?)

    private fun angGap(a: Pose, b: Pose): Double { val d = abs(a.az - b.az) % 360; return hypot(minOf(d, 360 - d), a.el - b.el) }

    /** A sticker is usable once seen in >= 3 shots spanning >= 15 degrees. */
    fun coverage(): List<Coverage> {
        val m = HashMap<Int, ArrayList<Pose?>>()
        shots.filter { it.state == "done" }.forEach { s -> s.stickers.forEach { st -> m.getOrPut(st.id) { ArrayList() }.add(s.pose) } }
        return m.entries.map { (id, poses) ->
            val ps = poses.filterNotNull()
            var spread = 0.0
            for (i in ps.indices) for (j in i + 1 until ps.size) spread = maxOf(spread, angGap(ps[i], ps[j]))
            val az = if (ps.isEmpty()) null else {
                val sx = ps.sumOf { Math.sin(Math.toRadians(it.az)) }; val cx = ps.sumOf { Math.cos(Math.toRadians(it.az)) }
                Math.toDegrees(Math.atan2(sx, cx))
            }
            Coverage(id, poses.size, poses.size >= 3 && (spread >= 15 || ps.size < 2), az)
        }.sortedBy { it.id }
    }

    fun sessionDoc(): JSONObject = JSONObject()
        .put("name", name).put("mode", mode).put("savePhotos", true).put("created", System.currentTimeMillis())
        .put("ua", "scan3d-android/0.1 " + android.os.Build.MODEL)
        .put("board", JSONObject().put("squares", JSONArray(listOf(7, 10))).put("square_m", Detector.SQUARE_M)
            .put("marker_m", Detector.MARKER_M).put("dict", "DICT_5X5_100"))
        .put("targets", JSONObject().put("dict", "DICT_4X4_250").put("marker_mm", Detector.TARGET_MM))
        .put("detect_max_px", Detector.DETECT_MAX.toInt()).put("coords", "pixels in sensor frame (EXIF orientation given)")
        .put("device", android.os.Build.MODEL)
}
