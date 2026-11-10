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
 * "Explained" is signed: only a board edge of the SAME sense (dark to light or light to dark) within 1.5 px and
 * at least as strong accounts for a step in the photo. Without the sign, the marker pattern (an edge every few
 * pixels) explained away most of an object's outline.
 *
 * Where the step is, exactly: the steepest point of the profile is NOT the edge when the two sides have different
 * slopes. A rounded plastic edge under a ceiling light has a brightness ramp on the object side, and that moves
 * the steepest point by (difference of slopes) x blur^2 / (step height): a third of a pixel outwards against a
 * white square, a sixth inwards against a black one. So the profile around the steepest point is fitted with what
 * it is, a step between two ramps seen through a Gaussian blur, and the step's position is taken from the fit.
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
    private const val FIT_HALF = 3.5         // the edge model is fitted over this many px to either side of the steepest point

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
        val search = (SEARCH_SIL * ratio / STEP).roundToInt()               // the edge is looked for within +- this many samples
        val half = search + ((FIT_HALF + 0.5) / STEP).roundToInt()          // the profile reaches further, for the model fit
        val n = 2 * half + 1
        val prof = FloatArray(n); val pred = FloatArray(n); val tmp = FloatArray(n)
        val dg = FloatArray(n); val dp = FloatArray(n)
        val hit = DoubleArray(2)
        val fit = EdgeFit()
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
            // not measured: only a bound. One full pixel out from the last pixel inside, so that the bound never lies
            // inside the outline it came from (half a pixel out, half of them did, and pulled the model inwards)
            val fallback = Point(cu + 1.0 * ratio * nx, cv + 1.0 * ratio * ny, false)
            if (levels == null) { out.add(fallback); continue }
            val c = levels.cell(xs, ys); val lo = levels.lo[c].toFloat(); val hi = levels.hi[c].toFloat()
            if (hi - lo < 30f) { out.add(fallback); continue }
            // intensity profile across the boundary (three parallel lines averaged) and the board's own prediction
            var okPred = true
            for (k in 0 until n) {
                val s = (k - half) * STEP
                val pu = cu + s * nx; val pv = cv + s * ny
                prof[k] = (bilinear(img, iw, pu, pv) + bilinear(img, iw, pu - 0.75 * ny, pv + 0.75 * nx) + bilinear(img, iw, pu + 0.75 * ny, pv - 0.75 * nx)) / 3f
                val t = if (proj.hit(pu, pv, hit)) tpl.sample(hit[0], hit[1]) else -1f
                if (t >= 0f) pred[k] = lo + (hi - lo) * t / 255f
                else if (abs(k - half) <= search) { okPred = false; break }      // no print to compare with where the edge is looked for
                else pred[k] = prof[k]                                              // beyond the search range: only needed for smoothing
            }
            if (!okPred) { out.add(fallback); continue }
            smooth(prof, tmp, kern, kr); smooth(pred, tmp, kern, kr)
            for (k in 1 until n - 1) { dg[k] = (prof[k + 1] - prof[k - 1]) / (2 * STEP).toFloat(); dp[k] = (pred[k + 1] - pred[k - 1]) / (2 * STEP).toFloat() }
            dg[0] = dg[1]; dg[n - 1] = dg[n - 2]; dp[0] = dp[1]; dp[n - 1] = dp[n - 2]
            // strongest step that the print does not explain, preferring positions close to the mask boundary
            var best = -1; var bestScore = 0.0
            for (k in half - search..half + search) {
                val g = dg[k]; val ag = abs(g)
                var un = ag                                           // the part of the photo's step that the print does not explain
                for (q in max(0, k - expl)..minOf(n - 1, k + expl)) {
                    val p = dp[q]
                    if (g * p <= 0f) continue                         // a board edge of the opposite sense explains nothing
                    val u = ag - 1.3f * abs(p)                        // same sense: explained up to 1.3 x its strength (shadows weaken, never strengthen)
                    if (u < un) un = u
                }
                val s = (k - half) * STEP
                val z = (s - 0.5 * ratio) / (2.5 * ratio)
                val score = un * exp(-0.5 * z * z)
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
            // the step between two ramps, fitted: its position, not the steepest point (see the class comment)
            val sFit = fit.position(prof, n, half, s)
            if (!sFit.isNaN()) s = sFit
            out.add(Point(cu + s * nx, cv + s * ny, true))
        }
        return out
    }

    /**
     * Least-squares fit of a blurred step between two ramps to a profile:
     *   f(x) = l0 + ml x + A Phi(x / sigma) + dm (x Phi(x / sigma) + sigma phi(x / sigma)),   x = s - s0
     * (the convolution of [l0 + ml x for x < 0, l0 + A + (ml + dm) x for x > 0] with a Gaussian of width sigma).
     * Unknowns: l0, ml, A, dm, s0, sigma. Gauss-Newton from the steepest point. Reusable buffers, not thread-safe.
     */
    class EdgeFit {
        private val th = DoubleArray(6); private val g = DoubleArray(6); private val jac = DoubleArray(6)
        private val hm = Array(6) { DoubleArray(6) }

        /**
         * Edge position (same coordinate as sPeak: px along the profile, 0 at index `half`), or NaN when the profile
         * around the steepest point is not a step between two ramps (another edge nearby, texture, too little contrast).
         */
        fun position(prof: FloatArray, n: Int, half: Int, sPeak: Double): Double {
            val k0 = ((sPeak - FIT_HALF) / STEP).roundToInt() + half; val k1 = ((sPeak + FIT_HALF) / STEP).roundToInt() + half
            if (k0 < 0 || k1 >= n) return Double.NaN
            val q = (1.0 / STEP).roundToInt()
            val y0 = prof[k0].toDouble(); val y1 = prof[k1].toDouble()
            val ml = (prof[k0 + q] - y0) / (q * STEP); val mr = (y1 - prof[k1 - q]) / (q * STEP)
            val lAt = y0 + ml * (sPeak - (k0 - half) * STEP); val rAt = y1 - mr * ((k1 - half) * STEP - sPeak)
            th[0] = lAt; th[1] = ml; th[2] = rAt - lAt; th[3] = mr - ml; th[4] = sPeak; th[5] = 1.0
            for (iter in 0 until 8) {
                for (a in 0 until 6) { g[a] = 0.0; java.util.Arrays.fill(hm[a], 0.0) }
                for (k in k0..k1) {
                    val x = (k - half) * STEP - th[4]; val z = x / th[5]; val cP = cdf(z); val dP = pdf(z)
                    val r = prof[k] - (th[0] + th[1] * x + th[2] * cP + th[3] * (x * cP + th[5] * dP))
                    jac[0] = 1.0; jac[1] = x; jac[2] = cP; jac[3] = x * cP + th[5] * dP
                    jac[4] = -(th[1] + th[2] * dP / th[5] + th[3] * cP)
                    jac[5] = -th[2] * x * dP / (th[5] * th[5]) + th[3] * dP
                    for (a in 0 until 6) { g[a] += jac[a] * r; for (b in a until 6) hm[a][b] += jac[a] * jac[b] }
                }
                for (a in 0 until 6) { for (b in 0 until a) hm[a][b] = hm[b][a]; hm[a][a] += 1e-3 * hm[a][a] + 1e-9 }
                val d = Linear.solve(hm, g) ?: return Double.NaN
                val ds = d[4].coerceIn(-0.3, 0.3)
                for (a in 0 until 6) th[a] += if (a == 4) ds else if (a == 5) d[5].coerceIn(-0.3, 0.3) else d[a]
                th[5] = th[5].coerceIn(0.35, 3.0)
                if (abs(ds) < 1e-4) break
            }
            var ss = 0.0
            for (k in k0..k1) {
                val x = (k - half) * STEP - th[4]; val z = x / th[5]; val cP = cdf(z)
                val r = prof[k] - (th[0] + th[1] * x + th[2] * cP + th[3] * (x * cP + th[5] * pdf(z)))
                ss += r * r
            }
            val rms = sqrt(ss / (k1 - k0 + 1))
            if (th[4].isNaN() || abs(th[4] - sPeak) > 0.6 || abs(th[2]) < 8.0 || rms > 0.06 * abs(th[2]) + 1.0) return Double.NaN
            return th[4]
        }

        private fun pdf(z: Double) = exp(-0.5 * z * z) / 2.5066282746310002
        private fun cdf(z: Double): Double {            // Abramowitz & Stegun 7.1.26, |error| < 1.5e-7
            val x = abs(z) / 1.4142135623730951
            val t = 1.0 / (1.0 + 0.3275911 * x)
            val e = 1.0 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * exp(-x * x)
            return if (z >= 0) 0.5 * (1 + e) else 0.5 * (1 - e)
        }
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
