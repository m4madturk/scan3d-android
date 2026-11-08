package ir.scan3d.app

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import ir.scan3d.app.engine.Camera
import ir.scan3d.app.engine.CameraPrior
import ir.scan3d.app.engine.Carve
import ir.scan3d.app.engine.FrameObs
import ir.scan3d.app.engine.MarchingCubes
import ir.scan3d.app.engine.Mesh
import ir.scan3d.app.engine.PlaneCut
import ir.scan3d.app.engine.Pose6
import ir.scan3d.app.engine.SilView
import ir.scan3d.app.engine.Silhouette
import ir.scan3d.app.engine.SolveResult
import ir.scan3d.app.engine.Solver
import ir.scan3d.app.engine.Vec3
import org.json.JSONObject
import org.opencv.core.Mat
import org.opencv.core.Size
import org.opencv.objdetect.CharucoBoard
import org.opencv.objdetect.Objdetect
import java.io.File
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/** Everything from captured shots to an STL, entirely on the phone. */
object Pipeline {
    class Report(val sizeMm: DoubleArray, val volumeCm3: Double, val views: Int, val planes: Int, val sigma0: Double,
                 val stickers: Int, val frameworkFrames: Int, val warnings: List<String>, val seconds: Double, val triangles: Int)

    private const val SIL_MAX = 1200     // silhouettes are computed on a downscaled copy (speed; accuracy limited by the hull anyway)
    private const val TPL_PX_PER_MM = 4.0

    /** The printed board as a byte image (0 = black), from the same OpenCV board definition. */
    val boardTemplate: Silhouette.BoardTemplate by lazy {
        val d = Objdetect.getPredefinedDictionary(Objdetect.DICT_5X5_100)
        val b = CharucoBoard(Size(7.0, 10.0), Detector.SQUARE_M.toFloat(), Detector.MARKER_M.toFloat(), d)
        val w = (175 * TPL_PX_PER_MM).toInt(); val h = (250 * TPL_PX_PER_MM).toInt()
        val m = Mat(); b.generateImage(Size(w.toDouble(), h.toDouble()), m, 0, 1)
        val buf = ByteArray(w * h); m.get(0, 0, buf); m.release()
        Silhouette.BoardTemplate(buf, w, h, TPL_PX_PER_MM)
    }

    fun frameObs(shots: List<Shot>): List<FrameObs> = shots.mapIndexed { i, s ->
        val st = HashMap<Int, DoubleArray>()
        s.stickers.forEach { st[it.id] = DoubleArray(8) { k -> it.corners[k].toDouble() } }
        val b = s.board
        FrameObs(i, b?.ids ?: IntArray(0), b?.pts?.let { p -> DoubleArray(p.size) { p[it].toDouble() } } ?: DoubleArray(0), st, true)
    }

    fun solve(shots: List<Shot>, w: Int, h: Int, prior: CameraPrior?, log: (String) -> Unit): SolveResult {
        val solver = Solver(Detector.SQUARE_M * 1000, Detector.SQUARES_X, Detector.TARGET_MM); solver.log = log
        return solver.solve(frameObs(shots), w, h, prior)
    }

    /**
     * Build the model. shots must all share one sensor size. Returns (mesh in mm, centred on xy and
     * resting on z = 0, report).
     */
    fun build(shots: List<Shot>, files: List<File>, convex: Boolean, prior: CameraPrior?, voxel: Double,
              progress: (String, Double) -> Unit, log: (String) -> Unit): Pair<Mesh, Report> {
        val t0 = System.currentTimeMillis()
        val done = shots.indices.filter { shots[it].state == "done" && shots[it].board != null }
        require(done.size >= 2) { "fewer than 2 shots with the board" }
        // all shots should be in the canonical sensor frame; if a few are not (older sessions), keep the largest group
        val groups = done.groupBy { shots[it].w to shots[it].h }
        val (w, h) = groups.maxByOrNull { it.value.size }!!.key
        val use = done.filter { shots[it].w == w && shots[it].h == h }
        if (use.size < done.size) log("${done.size - use.size} shot(s) have a different image size and are skipped")
        progress("حل مارکرها و دوربین…", 0.05)
        val geo = solve(use.map { shots[it] }, w, h, prior, log)

        // silhouettes
        val views = ArrayList<SilView>()
        val sc = min(1.0, SIL_MAX / max(w, h).toDouble())
        val sw = (w * sc).toInt(); val sh = (h * sc).toInt()
        val camS = Camera(geo.camera.f * sc, geo.camera.cx * sc, geo.camera.cy * sc, geo.camera.k1, geo.camera.k2)
        // stickers that sit ON the board plane inside the board are misread board markers, not object points:
        // a tangent plane there would carve away everything above the board
        val onBoard = geo.stickers.filter { abs(it.centre.z) < 1.0 && it.centre.x in 0.0..175.0 && it.centre.y in 0.0..250.0 }.map { it.id }.toSet()
        onBoard.forEach { id -> log("sticker $id lies on the board plane: probably a misread board marker, ignored") }
        var dropped = 0
        for ((k, ui) in use.withIndex()) {
            val pose = geo.poses[k] ?: continue
            progress("دورخط عکس ${k + 1} از ${use.size}…", 0.1 + 0.4 * k / use.size)
            val gray = loadGray(files[ui], sw, sh, shots[ui].fileRot) ?: continue
            val r = Silhouette.extract(gray, sw, sh, camS, pose, boardTemplate) ?: run { log("shot ${shots[ui].n}: no silhouette"); null } ?: continue
            // sanity: the stickers seen in this shot sit on the object, so their solved centres must project inside the mask
            val seen = shots[ui].stickers.map { it.id }.toSet()
            var tot = 0; var inside = 0
            for (st in geo.stickers) if (st.id in seen && st.id !in onBoard) {
                val pr = camS.project(pose.toCamera(st.centre)) ?: continue
                tot++
                val u = pr[0].toInt(); val v = pr[1].toInt()
                if (u in 0 until sw && v in 0 until sh) {
                    var hit = false
                    for (dv in -3..3) for (du in -3..3) { val uu = u + du; val vv = v + dv; if (uu in 0 until sw && vv in 0 until sh && r.mask[vv * sw + uu]) hit = true }
                    if (hit) inside++
                }
            }
            val area = r.area.toDouble() / (sw * sh)
            if (tot >= 3 && inside < 0.6 * tot) { dropped++; log("shot ${shots[ui].n}: silhouette misses the stickers ($inside/$tot inside, area %.1f%%); dropped".format(100 * area)); continue }
            log("shot ${shots[ui].n}: board verified %.0f%%, object/unknown area %.1f%% of image, stickers inside $inside/$tot".format(100 * r.solidity, 100 * area))
            views.add(SilView(sw, sh, r.mask, camS, pose))
        }
        if (dropped > 0) log("$dropped silhouette(s) dropped by the sticker check")
        require(views.size >= 2) { "fewer than 2 usable silhouettes" }

        // marker planes (z-up frame)
        val planes = geo.stickers.filter { it.id !in onBoard && it.views >= min(3, views.size) }.map {
            PlaneCut(Vec3(it.centre.x, it.centre.y, -it.centre.z), Vec3(it.normalOut.x, it.normalOut.y, -it.normalOut.z), if (convex) null else 20.0)
        }
        progress("تراش اولیه…", 0.55)
        // tolerate a few wrong silhouettes: a voxel is carved only when >= votes views see it outside
        var votes = max(1, Math.round(0.06 * views.size).toInt())
        val boardBounds = arrayOf(doubleArrayOf(-15.0, 190.0), doubleArrayOf(-15.0, 265.0), doubleArrayOf(0.0, 120.0))
        // where do the solved stickers sit? the object must contain them
        val stZ = geo.stickers.filter { it.id !in onBoard }.map { -it.centre.z }
        log("stickers z-up range %.1f .. %.1f mm; carve tolerance $votes of ${views.size} views".format(stZ.minOrNull() ?: 0.0, stZ.maxOrNull() ?: 0.0))
        var coarse = Carve.carve(views, boardBounds, 2.0, 0.5, planes, votes)
        Carve.removeSkirt(coarse, 4.5)                     // plain squares at z~0 are "unknown" in every view: ignore them for the bounds
        var b = Carve.insideBounds(coarse, 4.0)
        if (b == null) {
            val fr = Carve.insideFractions(views, boardBounds, 8.0)
            fr.forEachIndexed { i, f -> log("view ${i + 1}: %.1f%% of the board volume inside its silhouette".format(100 * f)) }
            votes = max(2, Math.round(0.25 * views.size).toInt())
            log("empty intersection; retrying with tolerance $votes")
            coarse = Carve.carve(views, boardBounds, 2.0, 0.5, planes, votes)
            Carve.removeSkirt(coarse, 4.5)
            b = Carve.insideBounds(coarse, 4.0)
        }
        if (b == null) throw IllegalStateException("object not found inside the silhouettes (see log: silhouettes do not intersect)")
        val span = DoubleArray(3) { b[it][1] - b[it][0] }
        log("object bounds x %.0f..%.0f y %.0f..%.0f z 0..%.0f mm".format(b[0][0], b[0][1], b[1][0], b[1][1], b[2][1]))
        val vox = max(voxel, Math.cbrt(span[0] * span[1] * span[2] / 1.0e7))    // cap at ~10 M voxels (grid + marching cubes ~ 100 MB)
        progress("تراش نهایی (${"%.2f".format(vox)} mm)…", 0.6)
        val g = Carve.carve(views, b, vox, 0.5, planes, votes) { p -> progress("تراش نهایی…", 0.6 + 0.3 * p) }
        val skirt = Carve.removeSkirt(g, 4.5)
        if (skirt > 0) log("removed base skirt: $skirt thin columns (< 4.5 mm)")
        progress("ساخت مش…", 0.92)
        val mesh = MarchingCubes.run(g)
        val bb = mesh.bounds()
        val centred = mesh.translated((-(bb[0][0] + bb[1][0]) / 2).toFloat(), (-(bb[0][1] + bb[1][1]) / 2).toFloat(), (-bb[0][2]).toFloat())
        val size = DoubleArray(3) { bb[1][it] - bb[0][it] }
        val rep = Report(size, mesh.volume() / 1000, views.size, planes.size, geo.sigma0, geo.stickers.size, geo.frameworkFrames, geo.warnings,
            (System.currentTimeMillis() - t0) / 1000.0, mesh.triangleCount)
        progress("تمام", 1.0)
        return centred to rep
    }

    /** Gray bytes of a JPEG, in SENSOR frame (EXIF ignored on purpose), downscaled to (w, h). */
    /** Gray image at (w, h) in the canonical frame: the stored pixels are rotated clockwise by `rot` first. */
    fun loadGray(f: File, w: Int, h: Int, rot: Int = 0): ByteArray? {
        val swap = rot == 90 || rot == 270
        val dw = if (swap) h else w; val dh = if (swap) w else h          // size of the stored (unrotated) image we need
        val o = BitmapFactory.Options(); o.inJustDecodeBounds = true; BitmapFactory.decodeFile(f.absolutePath, o)
        if (o.outWidth <= 0) return null
        if ((o.outWidth > o.outHeight) != (dw > dh)) return loadGray(f, w, h, 0)   // stored orientation disagrees with rot: trust the pixels
        var sample = 1; while (o.outWidth / (sample * 2) >= dw && o.outHeight / (sample * 2) >= dh) sample *= 2
        val o2 = BitmapFactory.Options(); o2.inSampleSize = sample; o2.inPreferredConfig = Bitmap.Config.ARGB_8888
        val bm = BitmapFactory.decodeFile(f.absolutePath, o2) ?: return null
        val scaled = if (bm.width != dw || bm.height != dh) Bitmap.createScaledBitmap(bm, dw, dh, true) else bm
        val px = IntArray(dw * dh); scaled.getPixels(px, 0, dw, 0, 0, dw, dh)
        val g = ByteArray(dw * dh) { val c = px[it]; ((((c shr 16) and 255) * 77 + ((c shr 8) and 255) * 150 + (c and 255) * 29) shr 8).toByte() }
        if (scaled !== bm) scaled.recycle(); bm.recycle()
        if (rot == 0) return g
        val out = ByteArray(w * h)
        for (y in 0 until dh) for (x in 0 until dw) {
            val (nx, ny) = when (rot) { 90 -> (dh - 1 - y) to x; 180 -> (dw - 1 - x) to (dh - 1 - y); else -> y to (dw - 1 - x) }
            out[ny * w + nx] = g[y * dw + x]
        }
        return out
    }

    fun reportJson(r: Report): JSONObject = JSONObject()
        .put("size_mm", org.json.JSONArray(r.sizeMm.toList())).put("volume_cm3", r.volumeCm3).put("views_used", r.views)
        .put("marker_planes", r.planes).put("sigma0_px", r.sigma0).put("stickers", r.stickers).put("framework_frames", r.frameworkFrames)
        .put("warnings", org.json.JSONArray(r.warnings)).put("seconds", r.seconds).put("triangles", r.triangles)
}
