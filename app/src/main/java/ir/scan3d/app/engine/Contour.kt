package ir.scan3d.app.engine

import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sqrt

/**
 * Sub-pixel object edges along a silhouette's boundary.
 *
 * A silhouette mask is only as sharp as its pixels (and the AI mask behind it is coarser still). The edge the
 * camera actually recorded is in the photo: next to every boundary pixel of the mask, the intensity profile
 * across the boundary is searched for a step that the printed board does NOT explain (the board is rendered
 * through the solved camera, so its own edges are known), and that step is located to a fraction of a pixel.
 *
 * Points whose edge is confirmed this way are measurements; the others (white object on a white square, a
 * boundary that is only "unknown") are just bounds: the object lies inside them.
 */
object Contour {
    /** A contour point in the analysis image (px) and whether a photographed object edge was found there. */
    class Point(@JvmField val u: Double, @JvmField val v: Double, @JvmField val edge: Boolean)

    private const val STEP = 0.25            // profile sampling (analysis px)
    private const val SEARCH_SIL = 4.0       // search this far to both sides of the mask boundary (silhouette px)
    private const val BLUR_SIGMA = 0.8       // smoothing of the profile (analysis px)

    /**
     * @param mask   silhouette (object component only) at sw x sh
     * @param sdf    its signed distance in px (positive inside)
     * @param valid  pixels that look at the printed area (same size)
     * @param levels local black / white levels of the board in the photo (same image geometry as the mask)
     * @param img    analysis image (gray bytes, iw x ih): the same photo at a higher resolution
     * @param cam    camera of the analysis image
     */
    fun extract(mask: BooleanArray, sdf: FloatArray, valid: BooleanArray, sw: Int, sh: Int, levels: Silhouette.Levels?,
                img: ByteArray, iw: Int, ih: Int, cam: Camera, pose: Pose6, warp: SheetWarp?, tpl: Silhouette.BoardTemplate): List<Point> {
        val out = ArrayList<Point>()
        val ratio = iw.toDouble() / sw
        val proj = SheetProjector(cam, pose, warp)
        val half = (SEARCH_SIL * ratio / STEP).roundToInt()
        val n = 2 * half + 1
        val prof = FloatArray(n); val pred = FloatArray(n); val tmp = FloatArray(n)
        val dg = FloatArray(n); val dp = FloatArray(n)
        val hit = DoubleArray(2)
        // Gaussian kernel for the 1-D smoothing of the profiles
        val ks = BLUR_SIGMA / STEP; val kr = (2.5 * ks).roundToInt()
        val kern = FloatArray(2 * kr + 1) { exp(-0.5 * ((it - kr) / ks) * ((it - kr) / ks)).toFloat() }
        val expl = (1.5 * ratio / STEP).roundToInt()          // a board edge this close explains a step
        for (ys in 3 until sh - 3 step 2) for (xs in 3 until sw - 3 step 2) {
            val i = ys * sw + xs
            if (!mask[i] || (mask[i - 1] && mask[i + 1] && mask[i - sw] && mask[i + sw])) continue      // boundary pixels only
            // outward normal from the distance field (it grows inwards)
            var nx = -(sdf[i + 2] - sdf[i - 2]).toDouble(); var ny = -(sdf[i + 2 * sw] - sdf[i - 2 * sw]).toDouble()
            val nn = sqrt(nx * nx + ny * ny); if (nn < 1e-3) continue
            nx /= nn; ny /= nn
            // stay away from the border of the print: there the boundary is not an object edge
            val ax = (xs + 6 * nx).roundToInt(); val ay = (ys + 6 * ny).roundToInt(); val bx = (xs - 6 * nx).roundToInt(); val by = (ys - 6 * ny).roundToInt()
            if (ax < 0 || ay < 0 || ax >= sw || ay >= sh || bx < 0 || by < 0 || bx >= sw || by >= sh || !valid[ay * sw + ax] || !valid[by * sw + bx]) continue
            // boundary pixel centre in the analysis image
            val cu = (xs + 0.5) * ratio - 0.5; val cv = (ys + 0.5) * ratio - 0.5
            val reach = half * STEP + 2
            if (cu < reach || cv < reach || cu > iw - 2 - reach || cv > ih - 2 - reach) continue
            val fallback = Point(cu + 0.5 * ratio * nx, cv + 0.5 * ratio * ny, false)   // pixel centre -> pixel border
            if (levels == null) { out.add(fallback); continue }
            val c = levels.cell(xs, ys); val lo = levels.lo[c].toFloat(); val hi = levels.hi[c].toFloat()
            if (hi - lo < 30f) { out.add(fallback); continue }
            // intensity profile across the boundary (three parallel lines averaged) and the board's own prediction
            var okPred = true
            for (k in 0 until n) {
                val s = (k - half) * STEP
                val pu = cu + s * nx; val pv = cv + s * ny
                prof[k] = (bilinear(img, iw, pu, pv) + bilinear(img, iw, pu - 0.75 * ny, pv + 0.75 * nx) + bilinear(img, iw, pu + 0.75 * ny, pv - 0.75 * nx)) / 3f
                if (!proj.hit(pu, pv, hit)) { okPred = false; break }
                val t = tpl.sample(hit[0], hit[1]); if (t < 0f) { okPred = false; break }
                pred[k] = lo + (hi - lo) * t / 255f
            }
            if (!okPred) { out.add(fallback); continue }
            smooth(prof, tmp, kern, kr); smooth(pred, tmp, kern, kr)
            for (k in 1 until n - 1) { dg[k] = (prof[k + 1] - prof[k - 1]) / (2 * STEP).toFloat(); dp[k] = abs(pred[k + 1] - pred[k - 1]) / (2 * STEP).toFloat() }
            dg[0] = dg[1]; dg[n - 1] = dg[n - 2]; dp[0] = dp[1]; dp[n - 1] = dp[n - 2]
            // strongest step that the print does not explain, preferring positions close to the mask boundary
            var best = -1; var bestScore = 0.0
            for (k in 2 until n - 2) {
                var e = 0f
                for (q in max(0, k - expl)..minOf(n - 1, k + expl)) if (dp[q] > e) e = dp[q]
                val s = (k - half) * STEP
                val z = (s - 0.5 * ratio) / (2.5 * ratio)
                val score = (abs(dg[k]) - 1.2f * e) * exp(-0.5 * z * z)
                if (score > bestScore) { bestScore = score; best = k }
            }
            if (best < 0 || bestScore < 0.03 * (hi - lo)) { out.add(fallback); continue }
            // the score is prior-weighted, which pulls its maximum towards the mask boundary (a third of a pixel in
            // tests): climb to the true peak of the gradient itself before interpolating
            var climb = 0
            while (climb < 8) {
                val here = abs(dg[best])
                if (best + 1 < n - 2 && abs(dg[best + 1]) > here) best++
                else if (best - 1 > 1 && abs(dg[best - 1]) > here) best--
                else break
                climb++
            }
            var s = (best - half) * STEP
            val a = abs(dg[best - 1]); val b = abs(dg[best]); val d = abs(dg[best + 1])
            val den = a - 2 * b + d
            if (b >= a && b >= d && abs(den) > 1e-6f) s += STEP * 0.5 * (a - d) / den          // parabola through the gradient peak
            out.add(Point(cu + s * nx, cv + s * ny, true))
        }
        return out
    }

    private fun smooth(a: FloatArray, tmp: FloatArray, kern: FloatArray, kr: Int) {
        val n = a.size
        for (i in 0 until n) {
            var acc = 0f; var wsum = 0f
            for (q in -kr..kr) { val j = i + q; if (j < 0 || j >= n) continue; val w = kern[q + kr]; acc += w * a[j]; wsum += w }
            tmp[i] = acc / wsum
        }
        System.arraycopy(tmp, 0, a, 0, n)
    }

    private fun bilinear(g: ByteArray, w: Int, x: Double, y: Double): Float {
        val x0 = x.toInt(); val y0 = y.toInt(); val ax = (x - x0).toFloat(); val ay = (y - y0).toFloat(); val o = y0 * w + x0
        val a = (g[o].toInt() and 255); val b = (g[o + 1].toInt() and 255); val c = (g[o + w].toInt() and 255); val d = (g[o + w + 1].toInt() and 255)
        return (a * (1 - ax) + b * ax) * (1 - ay) + (c * (1 - ax) + d * ax) * ay
    }
}
