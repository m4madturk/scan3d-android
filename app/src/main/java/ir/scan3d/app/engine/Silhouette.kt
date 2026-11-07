package ir.scan3d.app.engine

import kotlin.math.max
import kotlin.math.min

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

    fun extract(gray: ByteArray, w: Int, h: Int, cam: Camera, pose: Pose6, tpl: BoardTemplate, thresh: Int = 55, aiMask: BooleanArray? = null): Result? {
        val R = pose.R(); val t = pose.t
        val C = pose.centre()
        // expected board image and validity, per pixel, via the ray-plane intersection
        val expected = ByteArray(w * h); val valid = BooleanArray(w * h)
        for (v in 0 until h) for (u in 0 until w) {
            val n = cam.undistort(u.toDouble(), v.toDouble())
            // ray direction in world: R^T [xn, yn, 1]
            val dx = R[0][0] * n[0] + R[1][0] * n[1] + R[2][0]; val dy = R[0][1] * n[0] + R[1][1] * n[1] + R[2][1]; val dz = R[0][2] * n[0] + R[1][2] * n[1] + R[2][2]
            if (dz == 0.0) continue
            val s = -C.z / dz
            if (s <= 0) continue
            val X = C.x + s * dx; val Y = C.y + s * dy
            val tx = (X * tpl.pxPerMm).toInt(); val ty = (Y * tpl.pxPerMm).toInt()
            if (tx < 0 || ty < 0 || tx >= tpl.w || ty >= tpl.h) continue
            expected[v * w + u] = tpl.img[ty * tpl.w + tx]; valid[v * w + u] = true
        }
        // erode validity so the paper edge is not flagged
        val valid2 = erode(valid, w, h, 4)
        // photographed levels of printed black and white (medians)
        val blacks = ArrayList<Int>(); val whites = ArrayList<Int>()
        for (i in 0 until w * h step 7) if (valid2[i]) { val e = expected[i].toInt() and 255; val g = gray[i].toInt() and 255; if (e < 128) blacks.add(g) else whites.add(g) }
        if (blacks.size < 50 || whites.size < 50) return null
        val lo = median(blacks); val hi = median(whites)
        // predicted level, with +-3 px tolerance to misregistration: pixel fits if it is between min and max of the neighbourhood prediction
        val predLo = IntArray(w * h); val predHi = IntArray(w * h)
        val pred = IntArray(w * h) { lo + ((hi - lo) * ((expected[it].toInt() and 255) / 255.0)).toInt() }
        for (v in 0 until h) for (u in 0 until w) {
            var mn = Int.MAX_VALUE; var mx = Int.MIN_VALUE
            for (dv in -3..3) { val vv = v + dv; if (vv < 0 || vv >= h) continue; for (du in -3..3) { val uu = u + du; if (uu < 0 || uu >= w) continue; val p = pred[vv * w + uu]; if (p < mn) mn = p; if (p > mx) mx = p } }
            predLo[v * w + u] = mn; predHi[v * w + u] = mx
        }
        var raw = BooleanArray(w * h) { i -> valid2[i] && run { val g = gray[i].toInt() and 255; g > predHi[i] + thresh || g < predLo[i] - thresh } }
        raw = open(raw, w, h, 2)
        var closed = close(raw, w, h, max(7, (0.038 * min(w, h)).toInt()))
        if (aiMask != null) { val dil = dilate(aiMask, w, h, 2); for (i in closed.indices) closed[i] = closed[i] && dil[i] }
        val comp = largestComponent(closed, w, h) ?: return null
        val filled = fillHoles(comp, w, h)
        val area = filled.count { it }
        val hullArea = convexHullArea(filled, w, h)
        return Result(filled, if (hullArea > 0) area / hullArea else 0.0, area)
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
