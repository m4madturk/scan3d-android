package ir.scan3d.app.engine

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Object silhouette by comparing the photo with the printed board as the solved camera would see
 * it (port of silhouette.board_silhouette with pose). Pure Kotlin on a gray byte image.
 * Steps: render expected board (white/black, with the ArUco payloads treated as "either"),
 * map printed levels to photographed levels, flag pixels that fit neither nearby level,
 * morphological clean-up, largest component, hole filling.
 */
object Silhouette {
    class Result(val mask: BooleanArray, val solidity: Double, val area: Int)

    /** boardImage: the printed board as a gray byte image at pxPerMm resolution (0 = black). */
    class BoardTemplate(val img: ByteArray, val w: Int, val h: Int, val pxPerMm: Double)

    /**
     * Three-state classification against the rendered board:
     *  - SURE BOARD: the local structure of the board (square edges, marker bits) is observed where the
     *    render predicts it (normalised cross-correlation over a window of ~half a square). This is
     *    invariant to shading and shadows, and it does not need black/white level estimates.
     *  - OBJECT: strong level mismatch with the local black/white levels (dark object on white, white on black).
     *  - UNKNOWN: plain square interiors and everything outside the board (a white object on a white square
     *    looks like the square). Unknown counts as "inside" for carving: other views decide.
     * The returned mask is "inside or unknown" = NOT sure board; the visual hull intersection does the rest.
     */
    fun extract(gray: ByteArray, w: Int, h: Int, cam: Camera, pose: Pose6, tpl: BoardTemplate, thresh: Int = 55, aiMask: BooleanArray? = null): Result? {
        val R = pose.R(); val t = pose.t
        val C = pose.centre()
        // expected board image and validity, per pixel, via the ray-plane intersection
        val expected = ByteArray(w * h); val valid = BooleanArray(w * h)
        var sqPx = 0.0; var sqN = 0
        for (v in 0 until h) for (u in 0 until w) {
            val n = cam.undistort(u.toDouble(), v.toDouble())
            val dx = R[0][0] * n[0] + R[1][0] * n[1] + R[2][0]; val dy = R[0][1] * n[0] + R[1][1] * n[1] + R[2][1]; val dz = R[0][2] * n[0] + R[1][2] * n[1] + R[2][2]
            if (dz == 0.0) continue
            val s = -C.z / dz
            if (s <= 0) continue
            val X = C.x + s * dx; val Y = C.y + s * dy
            val tx = (X * tpl.pxPerMm).toInt(); val ty = (Y * tpl.pxPerMm).toInt()
            if (tx < 0 || ty < 0 || tx >= tpl.w || ty >= tpl.h) continue
            expected[v * w + u] = tpl.img[ty * tpl.w + tx]; valid[v * w + u] = true
        }
        val valid2 = erode(valid, w, h, 4)
        var nValid = 0; for (b in valid2) if (b) nValid++
        if (nValid < 2000) return null
        // projected size of one 25 mm square (px): from the camera distance and focal length
        val dist = sqrt(C.x * C.x + C.y * C.y + C.z * C.z).coerceAtLeast(50.0)
        val squarePx = 25.0 * cam.f / dist
        val win = max(9, (0.5 * squarePx).toInt()) or 1
        val r = win / 2

        // ---- structure test: NCC between the photo and the (slightly blurred) render ----
        val e = FloatArray(w * h) { (expected[it].toInt() and 255).toFloat() }
        val g = FloatArray(w * h) { (gray[it].toInt() and 255).toFloat() }
        val eb = boxMean(e, w, h, 1)                        // mimic the camera blur on the render
        val me = boxMean(eb, w, h, r); val mg = boxMean(g, w, h, r)
        val ee = boxMean(FloatArray(w * h) { eb[it] * eb[it] }, w, h, r)
        val gg = boxMean(FloatArray(w * h) { g[it] * g[it] }, w, h, r)
        val eg = boxMean(FloatArray(w * h) { eb[it] * g[it] }, w, h, r)
        var sure = BooleanArray(w * h)
        for (i in 0 until w * h) {
            if (!valid2[i]) continue
            val ve = ee[i] - me[i] * me[i]; val vg = gg[i] - mg[i] * mg[i]
            if (ve < 30f * 30f) continue                        // no board structure in this window: unknown
            val ncc = (eg[i] - me[i] * mg[i]) / sqrt(max(ve, 1e-3f) * max(vg, 1e-3f))
            if (ncc > 0.5f) sure[i] = true
        }
        // a window that straddles the object edge fails although its centre may still be board: tighten
        sure = dilate(sure, w, h, max(1, (0.1 * win).toInt()))

        // ---- level test for the parts the structure cannot judge (object over a plain square) ----
        val blacks = ArrayList<Int>(); val whites = ArrayList<Int>()
        for (i in 0 until w * h step 7) if (valid2[i]) { val ev = expected[i].toInt() and 255; val gv = gray[i].toInt() and 255; if (ev < 128) blacks.add(gv) else whites.add(gv) }
        if (blacks.size >= 50 && whites.size >= 50) {
            val gLo = median(blacks); val gHi = median(whites)
            val cells = 6; val cw = (w + cells - 1) / cells; val ch = (h + cells - 1) / cells
            val cellLo = IntArray(cells * cells) { gLo }; val cellHi = IntArray(cells * cells) { gHi }
            val cb = Array(cells * cells) { ArrayList<Int>() }; val cwh = Array(cells * cells) { ArrayList<Int>() }
            for (v in 0 until h step 3) for (u in 0 until w step 3) { val i = v * w + u; if (!valid2[i] || !sure[i]) continue
                val ev = expected[i].toInt() and 255; val gv = gray[i].toInt() and 255; val c = (v / ch) * cells + (u / cw)
                if (ev < 128) cb[c].add(gv) else cwh[c].add(gv) }
            for (c in 0 until cells * cells) { if (cb[c].size >= 40) cellLo[c] = median(cb[c]); if (cwh[c].size >= 40) cellHi[c] = median(cwh[c]) }
            for (c in 0 until cells * cells) if (cb[c].size < 40 || cwh[c].size < 40) {
                var best = -1; var bd = Int.MAX_VALUE
                for (d in 0 until cells * cells) if (cb[d].size >= 40 && cwh[d].size >= 40) { val dd = abs(d / cells - c / cells) + abs(d % cells - c % cells); if (dd < bd) { bd = dd; best = d } }
                if (best >= 0) { cellLo[c] = cellLo[best]; cellHi[c] = cellHi[best] }
            }
            val pred = IntArray(w * h); val thr = IntArray(w * h)
            for (v in 0 until h) for (u in 0 until w) { val c = (v / ch) * cells + (u / cw); val lo = cellLo[c]; val hi = cellHi[c]
                pred[v * w + u] = lo + ((hi - lo) * ((expected[v * w + u].toInt() and 255) / 255.0)).toInt()
                thr[v * w + u] = max(22, min(thresh, (0.33 * (hi - lo)).toInt())) }
            for (v in 0 until h) for (u in 0 until w) {
                val i = v * w + u; if (!valid2[i] || !sure[i]) continue
                var mn = Int.MAX_VALUE; var mx = Int.MIN_VALUE
                for (dv in -3..3) { val vv = v + dv; if (vv < 0 || vv >= h) continue; for (du in -3..3) { val uu = u + du; if (uu < 0 || uu >= w) continue; val p = pred[vv * w + uu]; if (p < mn) mn = p; if (p > mx) mx = p } }
                val gv = gray[i].toInt() and 255
                if (gv > mx + thr[i] || gv < mn - thr[i]) sure[i] = false            // strong mismatch: object, never board
            }
        }
        if (aiMask != null) { val dil = dilate(aiMask, w, h, 2); for (i in sure.indices) if (!dil[i] && valid2[i]) sure[i] = true }   // outside the AI mask is board
        val inside = BooleanArray(w * h) { !sure[it] }
        var nSure = 0; for (b in sure) if (b) nSure++
        var area = 0; for (i in 0 until w * h) if (valid2[i] && inside[i]) area++
        return Result(inside, nSure.toDouble() / nValid, area)
    }

    /** Separable box mean with radius r (window 2r+1), zero-padded at the borders by replication. */
    fun boxMean(src: FloatArray, w: Int, h: Int, r: Int): FloatArray {
        val tmp = FloatArray(w * h); val out = FloatArray(w * h)
        val n = (2 * r + 1).toFloat()
        for (v in 0 until h) {
            val row = v * w
            var acc = 0f
            for (u in -r..r) acc += src[row + u.coerceIn(0, w - 1)]
            tmp[row] = acc / n
            for (u in 1 until w) { acc += src[row + (u + r).coerceIn(0, w - 1)] - src[row + (u - r - 1).coerceIn(0, w - 1)]; tmp[row + u] = acc / n }
        }
        for (u in 0 until w) {
            var acc = 0f
            for (v in -r..r) acc += tmp[v.coerceIn(0, h - 1) * w + u]
            out[u] = acc / n
            for (v in 1 until h) { acc += tmp[(v + r).coerceIn(0, h - 1) * w + u] - tmp[(v - r - 1).coerceIn(0, h - 1) * w + u]; out[v * w + u] = acc / n }
        }
        return out
    }

    private fun median(a: List<Int>): Int { val s = a.sorted(); return s[s.size / 2] }

    private fun morph(m: BooleanArray, w: Int, h: Int, r: Int, dilate: Boolean): BooleanArray {
        // separable square structuring element, close enough to a disc for our sizes
        val tmp = BooleanArray(w * h); val out = BooleanArray(w * h)
        for (v in 0 until h) for (u in 0 until w) { var acc = !dilate
            for (du in -r..r) { val uu = u + du; if (uu < 0 || uu >= w) continue; val x = m[v * w + uu]; acc = if (dilate) acc || x else acc && x }
            tmp[v * w + u] = acc }
        for (v in 0 until h) for (u in 0 until w) { var acc = !dilate
            for (dv in -r..r) { val vv = v + dv; if (vv < 0 || vv >= h) continue; val x = tmp[vv * w + u]; acc = if (dilate) acc || x else acc && x }
            out[v * w + u] = acc }
        return out
    }
    fun dilate(m: BooleanArray, w: Int, h: Int, r: Int) = morph(m, w, h, r, true)
    fun erode(m: BooleanArray, w: Int, h: Int, r: Int) = morph(m, w, h, r, false)
    fun open(m: BooleanArray, w: Int, h: Int, r: Int) = dilate(erode(m, w, h, r), w, h, r)
    fun close(m: BooleanArray, w: Int, h: Int, r: Int) = erode(dilate(m, w, h, r), w, h, r)

    fun largestComponent(m: BooleanArray, w: Int, h: Int): BooleanArray? {
        val label = IntArray(w * h) { -1 }; var best = -1; var bestN = 0; var cur = 0
        val stack = IntArray(w * h)
        for (s in 0 until w * h) {
            if (!m[s] || label[s] >= 0) continue
            var sp = 0; stack[sp++] = s; label[s] = cur; var n = 0
            while (sp > 0) { val i = stack[--sp]; n++; val u = i % w; val v = i / w
                for ((du, dv) in arrayOf(1 to 0, -1 to 0, 0 to 1, 0 to -1)) { val uu = u + du; val vv = v + dv; if (uu < 0 || vv < 0 || uu >= w || vv >= h) continue
                    val j = vv * w + uu; if (m[j] && label[j] < 0) { label[j] = cur; stack[sp++] = j } } }
            if (n > bestN) { bestN = n; best = cur }
            cur++
        }
        if (best < 0) return null
        return BooleanArray(w * h) { label[it] == best }
    }

    fun fillHoles(m: BooleanArray, w: Int, h: Int): BooleanArray {
        // flood the background from the border; everything not reached and not object becomes object
        val bg = BooleanArray(w * h); val stack = IntArray(w * h); var sp = 0
        for (u in 0 until w) { for (v in intArrayOf(0, h - 1)) { val i = v * w + u; if (!m[i] && !bg[i]) { bg[i] = true; stack[sp++] = i } } }
        for (v in 0 until h) { for (u in intArrayOf(0, w - 1)) { val i = v * w + u; if (!m[i] && !bg[i]) { bg[i] = true; stack[sp++] = i } } }
        while (sp > 0) { val i = stack[--sp]; val u = i % w; val v = i / w
            for ((du, dv) in arrayOf(1 to 0, -1 to 0, 0 to 1, 0 to -1)) { val uu = u + du; val vv = v + dv; if (uu < 0 || vv < 0 || uu >= w || vv >= h) continue
                val j = vv * w + uu; if (!m[j] && !bg[j]) { bg[j] = true; stack[sp++] = j } } }
        return BooleanArray(w * h) { !bg[it] }
    }

    private fun convexHullArea(m: BooleanArray, w: Int, h: Int): Double {
        // boundary points -> Andrew monotone chain
        val pts = ArrayList<IntArray>()
        for (v in 0 until h) { var first = -1; var last = -1; for (u in 0 until w) if (m[v * w + u]) { if (first < 0) first = u; last = u }
            if (first >= 0) { pts.add(intArrayOf(first, v)); pts.add(intArrayOf(last, v)) } }
        if (pts.size < 3) return 0.0
        pts.sortWith(compareBy({ it[0] }, { it[1] }))
        fun cross(o: IntArray, a: IntArray, b: IntArray) = (a[0] - o[0]).toLong() * (b[1] - o[1]) - (a[1] - o[1]).toLong() * (b[0] - o[0])
        val hull = ArrayList<IntArray>()
        for (p in pts) { while (hull.size >= 2 && cross(hull[hull.size - 2], hull[hull.size - 1], p) <= 0) hull.removeAt(hull.size - 1); hull.add(p) }
        val lower = hull.size + 1
        for (i in pts.size - 2 downTo 0) { val p = pts[i]; while (hull.size >= lower && cross(hull[hull.size - 2], hull[hull.size - 1], p) <= 0) hull.removeAt(hull.size - 1); hull.add(p) }
        hull.removeAt(hull.size - 1)
        var a = 0.0
        for (i in hull.indices) { val p = hull[i]; val q = hull[(i + 1) % hull.size]; a += p[0].toDouble() * q[1] - q[0].toDouble() * p[1] }
        return Math.abs(a) / 2
    }
}
