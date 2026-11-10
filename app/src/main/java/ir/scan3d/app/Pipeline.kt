package ir.scan3d.app

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import ir.scan3d.app.engine.Camera
import ir.scan3d.app.engine.CameraPrior
import ir.scan3d.app.engine.Carve
import ir.scan3d.app.engine.Contour
import ir.scan3d.app.engine.FrameObs
import ir.scan3d.app.engine.MarchingCubes
import ir.scan3d.app.engine.Mesh
import ir.scan3d.app.engine.Parallel
import ir.scan3d.app.engine.PlaneCut
import ir.scan3d.app.engine.Pose6
import ir.scan3d.app.engine.RayFit
import ir.scan3d.app.engine.SheetWarp
import ir.scan3d.app.engine.SilView
import ir.scan3d.app.engine.Silhouette
import ir.scan3d.app.engine.SolveResult
import ir.scan3d.app.engine.Solver
import ir.scan3d.app.engine.Vec3
import org.json.JSONArray
import org.json.JSONObject
import org.opencv.core.Mat
import org.opencv.core.Size
import org.opencv.objdetect.CharucoBoard
import org.opencv.objdetect.Objdetect
import java.io.File
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

/** Everything from captured shots to an STL, entirely on the phone. */
object Pipeline {
    /** The clean parametric model and how well the photographed contours support it. */
    class Fit(val p: DoubleArray, val sigma: DoubleArray?, val tiltDeg: Double, val within: Double, val medianMm: Double,
              val rays: Int, val edges: Int, val views: Int, val stickers: Int)

    class Report(val sizeMm: DoubleArray, val volumeCm3: Double, val views: Int, val planes: Int, val sigma0: Double,
                 val stickers: Int, val frameworkFrames: Int, val warnings: List<String>, val seconds: Double, val triangles: Int,
                 val fit: Fit? = null, val sheetBendMm: Double = 0.0, val focalPx: Double = 0.0, val frameRms: DoubleArray = DoubleArray(0))

    /** The scan hull, the clean parametric model (when the contours fit one) and the numbers. */
    class Output(val mesh: Mesh, val clean: Mesh?, val report: Report)

    /** Below this fraction of confirmed contour edges within RayFit.TOL_MM of the model, the object is not a rounded box. */
    const val FIT_MIN = 0.40

    private const val SIL_MAX = 1200          // silhouettes (masks) are computed on a downscaled copy
    private const val ANALYSIS_MAX = 2400     // contour edges are measured on this larger copy (phone pixels are noisy at 1:1)
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
        val solver = Solver(Detector.SQUARE_M * 1000, Detector.SQUARES_X, Detector.TARGET_MM, Detector.SQUARES_Y); solver.log = log
        return solver.solve(frameObs(shots), w, h, prior)
    }

    /** Camera of an image that is the full frame resampled to `scale` of its size (pixel centres handled). */
    private fun scaled(c: Camera, scale: Double) = Camera(c.f * scale, (c.cx + 0.5) * scale - 0.5, (c.cy + 0.5) * scale - 0.5, c.k1, c.k2)

    /** What one photo contributes: its silhouette for carving and its measured contour points for the model fit. */
    private class ViewOut(val sil: SilView?, val contour: List<Contour.Point>, val camA: Camera?, val pose: Pose6?, val note: String, val ai: Int)

    /**
     * Build the model. shots must all share one sensor size. Returns the mesh in mm (centred on xy, resting on
     * z = 0), the clean model when there is one, and the report.
     */
    fun build(shots: List<Shot>, files: List<File>, convex: Boolean, prior: CameraPrior?, voxel: Double,
              progress: (String, Double) -> Unit, log: (String) -> Unit, segmenter: Segmenter? = null): Output {
        val t0 = System.currentTimeMillis()
        var tStage = t0
        fun stage(name: String) { val now = System.currentTimeMillis(); log("[time] $name: ${now - tStage} ms"); tStage = now }
        val done = shots.indices.filter { shots[it].state == "done" && shots[it].board != null }
        require(done.size >= 2) { "fewer than 2 shots with the board" }
        // all shots should be in the canonical sensor frame; if a few are not (older sessions), keep the largest group
        val groups = done.groupBy { shots[it].w to shots[it].h }
        val (w, h) = groups.maxByOrNull { it.value.size }!!.key
        val use = done.filter { shots[it].w == w && shots[it].h == h }
        if (use.size < done.size) log("${done.size - use.size} shot(s) have a different image size and are skipped")
        progress("حل مارکرها و دوربین…", 0.03)
        val geo = solve(use.map { shots[it] }, w, h, prior, log)
        val warp = geo.warp
        stage("solve")

        // stickers that sit ON the board plane inside the board are misread board markers, not object points:
        // a tangent plane there would carve away everything above the board
        val onBoard = geo.stickers.filter { abs(it.centre.z) < 1.0 && it.centre.x in 0.0..175.0 && it.centre.y in 0.0..250.0 }.map { it.id }.toSet()
        onBoard.forEach { id -> log("sticker $id lies on the board plane: probably a misread board marker, ignored") }
        val objStickers = geo.stickers.filter { it.id !in onBoard }
        val objCentres = objStickers.map { it.centre }

        // ---- per photo: silhouette (for the hull) and sub-pixel contour edges (for the clean model), in parallel ----
        val sc = min(1.0, SIL_MAX / max(w, h).toDouble())
        val sw = (w * sc).toInt(); val sh = (h * sc).toInt()
        val camS = scaled(geo.camera, sw.toDouble() / w)
        val outs = arrayOfNulls<ViewOut>(use.size)
        val finished = AtomicInteger(0)
        val heapMb = Runtime.getRuntime().maxMemory() / (1 shl 20)
        val workers = min(Parallel.threads, if (heapMb >= 450) 3 else if (heapMb >= 300) 2 else 1)
        val tpl = boardTemplate
        Parallel.forEach(use.size, workers) { k ->
            val ui = use[k]; val shot = shots[ui]
            val pose = geo.poses[k]
            outs[k] = if (pose == null) ViewOut(null, emptyList(), null, null, "shot ${shot.n}: no pose", 0)
                      else processView(shot, files[ui], w, h, sw, sh, camS, geo.camera, pose, warp, tpl, objCentres, objStickers.map { it.id to it.centre }, segmenter)
            val n = finished.incrementAndGet()
            synchronized(finished) { progress("دورخط و لبه‌یابی عکس $n از ${use.size}…", 0.08 + 0.42 * n / use.size) }
        }
        val views = ArrayList<SilView>(); var aiOk = 0; var aiFallback = 0; var dropped = 0
        for (o in outs) { o ?: continue; log(o.note); if (o.ai > 0) aiOk++ else if (o.ai < 0) aiFallback++; if (o.sil != null) views.add(o.sil) else dropped++ }
        if (segmenter != null) log("AI segmentation: $aiOk views refined, $aiFallback fell back to the structure test")
        if (dropped > 0) log("$dropped photo(s) gave no usable silhouette")
        require(views.size >= 2) { "fewer than 2 usable silhouettes" }
        stage("silhouettes + contours (${workers} threads)")

        // ---- visual hull (the raw scan) ----
        val planes = objStickers.filter { it.views >= min(3, views.size) }.map {
            PlaneCut(Vec3(it.centre.x, it.centre.y, -it.centre.z), Vec3(it.normalOut.x, it.normalOut.y, -it.normalOut.z), if (convex) null else 20.0)
        }
        progress("تراش اولیه…", 0.52)
        // tolerate a few wrong silhouettes: a voxel is carved only when >= votes views see it outside
        var votes = max(1, Math.round(0.06 * views.size).toInt())
        // only above the board itself: outside it every view is "unknown" and nothing could ever be carved there
        val boardBounds = arrayOf(doubleArrayOf(0.0, 175.0), doubleArrayOf(0.0, 250.0), doubleArrayOf(0.0, 120.0))
        val stZ = objStickers.map { -it.centre.z }
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
        progress("تراش نهایی (${"%.2f".format(vox)} mm)…", 0.56)
        val g = Carve.carve(views, b, vox, 0.5, planes, votes) { p -> progress("تراش نهایی…", 0.56 + 0.24 * p) }
        val skirt = Carve.removeSkirt(g, 4.5)
        if (skirt > 0) log("removed base skirt: $skirt thin columns (< 4.5 mm)")
        stage("carving (${Parallel.threads} threads, voxel %.2f mm)".format(vox))
        progress("ساخت مش…", 0.82)
        val raw = MarchingCubes.run(g).welded()
        val mesh = raw.largestComponent().taubin(12)        // drop loose crumbs, take the voxel staircase off (volume preserved)
        if (mesh.triangleCount < raw.triangleCount) log("removed ${raw.triangleCount - mesh.triangleCount} triangles of loose pieces")
        val bb = mesh.bounds()
        val dx = (-(bb[0][0] + bb[1][0]) / 2).toFloat(); val dy = (-(bb[0][1] + bb[1][1]) / 2).toFloat(); val dz = (-bb[0][2]).toFloat()
        val centred = mesh.translated(dx, dy, dz)
        val size = DoubleArray(3) { bb[1][it] - bb[0][it] }
        stage("mesh (${mesh.triangleCount} triangles)")

        // ---- clean parametric model, fitted to the measured contour edges ----
        progress("برازش مدل تمیز روی لبه‌های عکس…", 0.88)
        val warnings = ArrayList(geo.warnings)
        var clean: Mesh? = null; var fitOut: Fit? = null
        try {
            val p0 = RayFit.initFromPoints(mesh.verts)
            if (p0 == null) log("clean model: too little surface above the base")
            else {
                val zs = warp.height(p0[RayFit.CX], p0[RayFit.CY])
                val rays = RayFit.Rays(4096)
                for ((k, o) in outs.withIndex()) { o ?: continue; val cam = o.camA ?: continue; val pose = o.pose ?: continue
                    val R = pose.R(); val c = pose.centre()
                    for (pt in o.contour) rays.addPixel(cam, R, c, pt.u, pt.v, pt.edge, k, zs) }
                val good = objStickers.filter { it.views >= 2 && (it.sigma.maxOrNull() ?: 0.0).let { s -> s.isNaN() || s < 0.5 } }
                val st = DoubleArray(good.size * 3); good.forEachIndexed { i, s -> st[3 * i] = s.centre.x; st[3 * i + 1] = s.centre.y; st[3 * i + 2] = -s.centre.z }
                val r = RayFit.fit(p0, rays, st, zs, true, log)
                if (r == null) log("clean model: only ${rays.n} contour rays, not enough")
                else {
                    val p = RayFit.canonical(r.p)
                    log("clean model: %.2f x %.2f x %.2f mm, corner R %.1f, top edge R %.1f, bottom edge R %.1f, yaw %.1f deg, tilt %.1f deg".format(
                        p[RayFit.W], p[RayFit.L], p[RayFit.H], p[RayFit.RC], p[RayFit.RZ], p[RayFit.RB], Math.toDegrees(p[RayFit.TH]), r.tiltDeg))
                    r.sigma?.let { s -> log("  repeatability (leave one photo out): W %.2f  L %.2f  H %.2f  corner R %.2f mm".format(s[RayFit.W], s[RayFit.L], s[RayFit.H], s[RayFit.RC])) }
                    if (r.stickerResiduals.isNotEmpty()) log("  sticker centres vs surface (mm): " + r.stickerResiduals.joinToString(" ") { "%+.2f".format(it) })
                    if (r.within >= FIT_MIN && r.edges >= 150 && r.views >= 4) {
                        clean = RayFit.mesh(p).translated(dx, dy, 0f)
                        // sigma in canonical order: W/L may have been swapped
                        val sg = r.sigma?.let { s -> if (r.p[RayFit.L] > r.p[RayFit.W]) s.copyOf().also { c -> c[RayFit.W] = s[RayFit.L]; c[RayFit.L] = s[RayFit.W] } else s }
                        fitOut = Fit(p, sg, r.tiltDeg, r.within, r.medianAbs, r.rays, r.edges, r.views, good.size)
                        if (r.tiltDeg > 2.0) warnings.add("object tilted %.1f deg on the sheet: it is not lying flat, so its height is uncertain by about %.1f mm".format(
                            r.tiltDeg, 0.5 * max(p[RayFit.W], p[RayFit.L]) * Math.sin(Math.toRadians(r.tiltDeg))))
                    } else log("clean model rejected: %.0f%% of %d edges within %.1f mm (need %.0f%%, 150 edges, 4 views)".format(100 * r.within, r.edges, RayFit.TOL_MM, 100 * FIT_MIN))
                }
            }
        } catch (e: Exception) { log("clean model failed: $e") }
        stage("clean model fit")
        val bend = warp.peakToValley()
        if (bend > 1.0) warnings.add("the sheet is bent by %.1f mm (it was measured and modelled; a rigid flat board is still better)".format(bend))
        val rep = Report(size, mesh.volume() / 1000, views.size, planes.size, geo.sigma0, geo.stickers.size, geo.frameworkFrames, warnings,
            (System.currentTimeMillis() - t0) / 1000.0, mesh.triangleCount, fitOut, bend, geo.camera.f, geo.frameRms)
        progress("تمام", 1.0)
        return Output(centred, clean, rep)
    }

    /** One photo: silhouette (structure test + AI), sanity check against the stickers, contour edges. Thread-safe. */
    private fun processView(shot: Shot, file: File, w: Int, h: Int, sw: Int, sh: Int, camS: Camera, camFull: Camera, pose: Pose6, warp: SheetWarp,
                            tpl: Silhouette.BoardTemplate, objCentres: List<Vec3>, stickers: List<Pair<Int, Vec3>>, segmenter: Segmenter?): ViewOut {
        val im = loadImages(file, w, h, sw, sh, shot.fileRot) ?: return ViewOut(null, emptyList(), null, null, "shot ${shot.n}: image could not be read", 0)
        val r = Silhouette.extract(im.sil, sw, sh, camS, pose, tpl, warp = warp) ?: return ViewOut(null, emptyList(), null, null, "shot ${shot.n}: no silhouette (board not visible enough)", 0)
        val uv = objCentres.mapNotNull { c -> camS.project(pose.toCamera(c)) }
        // AI refinement: segment a crop centred on the projected stickers; accept only if every sticker lands inside
        var aiNote = ""; var ai = 0
        if (segmenter != null && segmenter.available && uv.isNotEmpty()) {
            val cx = uv.sumOf { it[0] } / uv.size; val cy = uv.sumOf { it[1] } / uv.size
            val spread = uv.maxOf { hypot(it[0] - cx, it[1] - cy) }
            val rad = max(0.15 * max(sw, sh), 1.8 * spread + 0.08 * max(sw, sh)).toInt()
            val x0 = max(0, (cx - rad).toInt()); val y0 = max(0, (cy - rad).toInt())
            val cw = min(sw, (cx + rad).toInt()) - x0; val chh = min(sh, (cy + rad).toInt()) - y0
            val am = if (cw > 20 && chh > 20) synchronized(segmenter) { segmenter.maskCrop(im.sil, sw, sh, x0, y0, cw, chh) } else null
            if (am != null) {
                var nOn = 0; for (v in am) if (v) nOn++
                val frac = nOn.toDouble() / (cw * chh)
                val labels = Silhouette.components(am, sw, sh)
                val keep = uv.mapNotNull { p -> val u = p[0].toInt(); val v = p[1].toInt(); if (u in 0 until sw && v in 0 until sh && labels[v * sw + u] != 0) labels[v * sw + u] else null }.toSet()
                var allIn = keep.isNotEmpty()
                for (p in uv) { val u = p[0].toInt(); val v = p[1].toInt(); if (!(u in 0 until sw && v in 0 until sh && am[v * sw + u])) allIn = false }
                if (frac in 0.02..0.6 && allIn) {
                    val kept = BooleanArray(sw * sh) { labels[it] in keep }
                    val dil = Silhouette.dilate(kept, sw, sh, 2)
                    for (i in r.mask.indices) r.mask[i] = r.mask[i] && dil[i]
                    ai = 1; aiNote = ", AI ${pct(frac)} of crop"
                } else { ai = -1; aiNote = ", AI rejected (${pct(frac)} of crop, stickers inside: $allIn)" }
            } else { ai = -1; aiNote = ", AI unavailable" }
        }
        // sanity: the stickers seen in this shot sit on the object, so their solved centres must project inside the mask
        val seen = shot.stickers.map { it.id }.toSet()
        var tot = 0; var inside = 0
        for ((id, centre) in stickers) if (id in seen) {
            val pr = camS.project(pose.toCamera(centre)) ?: continue
            tot++
            val u = pr[0].toInt(); val v = pr[1].toInt()
            if (u in 0 until sw && v in 0 until sh) {
                var hit = false
                for (dv in -3..3) for (du in -3..3) { val uu = u + du; val vv = v + dv; if (uu in 0 until sw && vv in 0 until sh && r.mask[vv * sw + uu]) hit = true }
                if (hit) inside++
            }
        }
        val area = r.area.toDouble() / (sw * sh)
        if (tot >= 3 && inside < 0.6 * tot)
            return ViewOut(null, emptyList(), null, null, "shot ${shot.n}: silhouette misses the stickers ($inside/$tot inside, area ${pct(area)}); dropped", ai)
        val view = SilView(sw, sh, r.mask, camS, pose)
        // contour edges: only on the piece of the mask that holds the stickers (the object), measured on the larger image
        var contour: List<Contour.Point> = emptyList(); var camA: Camera? = null
        if (uv.isNotEmpty()) {
            val labels = Silhouette.components(r.mask, sw, sh)
            val keep = uv.mapNotNull { p -> val u = p[0].toInt(); val v = p[1].toInt(); if (u in 0 until sw && v in 0 until sh && labels[v * sw + u] != 0) labels[v * sw + u] else null }.toSet()
            if (keep.isNotEmpty()) {
                val obj = BooleanArray(sw * sh) { labels[it] in keep }
                var n = 0; for (v in obj) if (v) n++
                if (n < 0.35 * sw * sh) {                         // a mask that covers a third of the photo is not an object outline
                    val objView = SilView(sw, sh, obj, camS, pose)
                    val ca = scaled(camFull, im.aw.toDouble() / w)
                    camA = ca
                    contour = Contour.extract(obj, objView.sdf, r.valid, sw, sh, r.levels, im.ana, im.aw, im.ah, ca, pose, warp, tpl)
                }
            }
        }
        val edges = contour.count { it.edge }
        // NOTE: never run String.format over text that already contains a formatted "%": "12% of crop" parses as the
        // format specifier "% o" and throws (this crashed every build in which the AI mask was used)
        val note = "shot ${shot.n}: board verified ${pct(r.solidity)}, object/unknown area ${pct(area)} of image, stickers inside $inside/$tot$aiNote, contour ${contour.size} points ($edges measured edges)"
        return ViewOut(view, contour, camA, pose, note, ai)
    }

    private fun pct(x: Double) = "%.1f".format(100 * x) + "%"

    /** The photo as gray bytes in the canonical frame at two sizes: analysis (aw x ah) and silhouette (sw x sh). */
    class Images(val sil: ByteArray, val ana: ByteArray, val aw: Int, val ah: Int)

    /**
     * Decodes the JPEG once (subsampled so that the long side is at most ANALYSIS_MAX), rotates it clockwise by
     * `rot` into the canonical sensor frame and returns it together with a copy scaled to (sw, sh).
     * (w, h) is the canonical full size, used to detect a stored orientation that disagrees with `rot`.
     */
    fun loadImages(f: File, w: Int, h: Int, sw: Int, sh: Int, rot: Int = 0): Images? {
        val o = BitmapFactory.Options(); o.inJustDecodeBounds = true; BitmapFactory.decodeFile(f.absolutePath, o)
        if (o.outWidth <= 0) return null
        var r = rot
        val swap = r == 90 || r == 270
        if ((o.outWidth > o.outHeight) != ((if (swap) h else w) > (if (swap) w else h))) r = 0      // stored orientation disagrees with rot: trust the pixels
        var sample = 1; while (max(o.outWidth, o.outHeight) / sample > ANALYSIS_MAX) sample *= 2
        val o2 = BitmapFactory.Options(); o2.inSampleSize = sample; o2.inPreferredConfig = Bitmap.Config.ARGB_8888
        val bm = BitmapFactory.decodeFile(f.absolutePath, o2) ?: return null
        val ana = toGray(bm, r)
        val turned = r == 90 || r == 270
        val aw = if (turned) bm.height else bm.width; val ah = if (turned) bm.width else bm.height
        val dw = if (turned) sh else sw; val dh = if (turned) sw else sh                       // size of the stored (unrotated) silhouette image
        val small = if (bm.width != dw || bm.height != dh) Bitmap.createScaledBitmap(bm, dw, dh, true) else bm
        val sil = toGray(small, r)
        if (small !== bm) small.recycle(); bm.recycle()
        return Images(sil, ana, aw, ah)
    }

    /** Gray bytes of a bitmap, rotated clockwise by rot (0 / 90 / 180 / 270). */
    private fun toGray(bm: Bitmap, rot: Int): ByteArray {
        val dw = bm.width; val dh = bm.height
        val px = IntArray(dw * dh); bm.getPixels(px, 0, dw, 0, 0, dw, dh)
        val g = ByteArray(dw * dh) { val c = px[it]; ((((c shr 16) and 255) * 77 + ((c shr 8) and 255) * 150 + (c and 255) * 29) shr 8).toByte() }
        if (rot == 0) return g
        val w = if (rot == 180) dw else dh
        val out = ByteArray(dw * dh)
        for (y in 0 until dh) for (x in 0 until dw) {
            val nx: Int; val ny: Int
            when (rot) { 90 -> { nx = dh - 1 - y; ny = x }; 180 -> { nx = dw - 1 - x; ny = dh - 1 - y }; else -> { nx = y; ny = dw - 1 - x } }
            out[ny * w + nx] = g[y * dw + x]
        }
        return out
    }

    fun reportJson(r: Report): JSONObject {
        val j = JSONObject()
            .put("size_mm", JSONArray(r.sizeMm.toList())).put("volume_cm3", r.volumeCm3).put("views_used", r.views)
            .put("marker_planes", r.planes).put("sigma0_px", r.sigma0).put("stickers", r.stickers).put("framework_frames", r.frameworkFrames)
            .put("warnings", JSONArray(r.warnings)).put("seconds", r.seconds).put("triangles", r.triangles)
            .put("sheet_bend_mm", r.sheetBendMm).put("focal_px", r.focalPx)
            .put("frame_rms_px", JSONArray(r.frameRms.map { if (it.isNaN()) -1.0 else it }))
        r.fit?.let { f ->
            val p = f.p
            val o = JSONObject().put("w", p[RayFit.W]).put("l", p[RayFit.L]).put("h", p[RayFit.H]).put("rc", p[RayFit.RC]).put("rz", p[RayFit.RZ]).put("rb", p[RayFit.RB])
                .put("yaw_deg", Math.toDegrees(p[RayFit.TH])).put("tilt_deg", f.tiltDeg).put("within", f.within).put("median_mm", f.medianMm)
                .put("rays", f.rays).put("edges", f.edges).put("views", f.views).put("stickers", f.stickers)
            f.sigma?.let { s -> o.put("sigma_w", s[RayFit.W]).put("sigma_l", s[RayFit.L]).put("sigma_h", s[RayFit.H]).put("sigma_rc", s[RayFit.RC]) }
            j.put("fit", o)
        }
        return j
    }
}
