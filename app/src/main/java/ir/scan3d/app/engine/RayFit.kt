package ir.scan3d.app.engine

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * The clean model: a parametric rounded box fitted DIRECTLY to the photographed contours.
 *
 * A viewing ray through a point on the object's outline is tangent to the object, so the minimum of the
 * model's signed distance along that ray must be zero. Residual = that minimum in mm (> 0: the ray misses
 * the model, < 0: it cuts through it). Unlike fitting a box to the visual hull, this has no outward bias
 * from a finite number of views: with the 9 poses of a real scan the hull fit read 67.2 mm for a 66.0 mm
 * ideal box, the tangent-ray fit 66.0.
 *
 * Model (z-up frame, mm; the sheet is z = zs under the object):
 *   p = [cx, cy, theta, W, L, H, rc, rz, tx, ty, rb]
 *   footprint W x L with vertical-edge radius rc, height H, top-edge radius rz, bottom-edge radius rb,
 *   yaw theta, tilt (tx, ty). The body RESTS on the sheet: its lowest point touches it, so the vertical
 *   position is not a free parameter.
 * Sticker centres (triangulated to ~0.1 mm) are added as points that must lie on the surface.
 */
object RayFit {
    const val NP = 11
    const val CX = 0; const val CY = 1; const val TH = 2; const val W = 3; const val L = 4; const val H = 5
    const val RC = 6; const val RZ = 7; const val TX = 8; const val TY = 9; const val RB = 10

    private const val STICKER_T = 0.10     // sticker thickness: its printed face sits this far outside the surface (mm)
    private const val STICKER_W = 30.0     // one sticker centre counts like this many contour rays
    private const val STICKER_C = 0.6      // robust scale for stickers (one may sit on a feature that is not part of the box)
    private const val UNCONFIRMED = 0.05   // weight of a ray that misses the model when its contour point is only a bound
    private const val RB_PRIOR = 1.0       // weak pull of the bottom radius towards the top radius (it is rarely seen well)
    private val SCHEDULE = doubleArrayOf(4.0, 2.0, 1.0, 0.5)   // robust scale (mm), coarse to fine
    private val STEPS = doubleArrayOf(0.02, 0.02, Math.toRadians(0.1), 0.02, 0.02, 0.02, 0.02, 0.02, Math.toRadians(0.1), Math.toRadians(0.1), 0.02)
    const val MAX_RAYS = 4000
    const val TOL_MM = 0.5

    /** Contour rays, z-up frame. t1 = ray parameter where it meets the sheet (the search ends there). */
    class Rays(cap: Int = 1024) {
        var n = 0; private set
        var ox = DoubleArray(cap); var oy = DoubleArray(cap); var oz = DoubleArray(cap)
        var dx = DoubleArray(cap); var dy = DoubleArray(cap); var dz = DoubleArray(cap)
        var t1 = DoubleArray(cap); var edge = BooleanArray(cap); var view = IntArray(cap)

        fun add(oX: Double, oY: Double, oZ: Double, dX: Double, dY: Double, dZ: Double, isEdge: Boolean, viewIndex: Int, zs: Double) {
            if (n == ox.size) { val c = n * 2
                ox = ox.copyOf(c); oy = oy.copyOf(c); oz = oz.copyOf(c); dx = dx.copyOf(c); dy = dy.copyOf(c); dz = dz.copyOf(c); t1 = t1.copyOf(c); edge = edge.copyOf(c); view = view.copyOf(c) }
            val nn = sqrt(dX * dX + dY * dY + dZ * dZ)
            ox[n] = oX; oy[n] = oY; oz[n] = oZ; dx[n] = dX / nn; dy[n] = dY / nn; dz[n] = dZ / nn
            val dzn = dZ / nn
            t1[n] = if (dzn < -1e-6) ((zs - oZ) / dzn).coerceIn(1.0, 5000.0) else 5000.0
            edge[n] = isEdge; view[n] = viewIndex; n++
        }

        /** Adds the viewing ray of pixel (u, v) of a camera at `pose` (board frame, z into the board). */
        fun addPixel(cam: Camera, R: Array<DoubleArray>, centre: Vec3, u: Double, v: Double, isEdge: Boolean, viewIndex: Int, zs: Double) {
            val nrm = cam.undistort(u, v)
            val x = R[0][0] * nrm[0] + R[1][0] * nrm[1] + R[2][0]; val y = R[0][1] * nrm[0] + R[1][1] * nrm[1] + R[2][1]; val z = R[0][2] * nrm[0] + R[1][2] * nrm[1] + R[2][2]
            add(centre.x, centre.y, -centre.z, x, y, -z, isEdge, viewIndex, zs)          // flip z: board frame -> z-up
        }

        /** At most `limit` rays, evenly thinned. */
        fun thinned(limit: Int): Rays {
            if (n <= limit) return this
            val out = Rays(limit + 8); val stride = n.toDouble() / limit
            var q = 0.0
            while (q < n) { val i = q.toInt(); out.add(ox[i], oy[i], oz[i], dx[i], dy[i], dz[i], edge[i], view[i], 0.0); out.t1[out.n - 1] = t1[i]; q += stride }
            return out
        }
    }

    fun clamp(a: DoubleArray): DoubleArray {
        val p = if (a.size >= NP) a.copyOf() else a.copyOf(NP).also { it[RB] = 0.05 }
        p[W] = max(p[W], 2.0); p[L] = max(p[L], 2.0); p[H] = max(p[H], 1.0)
        val half = min(p[W], p[L]) / 2
        p[RZ] = p[RZ].coerceIn(0.05, max(0.06, min(p[H], half) - 0.1))
        p[RB] = p[RB].coerceIn(0.05, max(0.06, min(p[H] - p[RZ], half) - 0.05))
        p[RC] = p[RC].coerceIn(max(p[RZ], p[RB]), max(max(p[RZ], p[RB]), half - 0.05))
        p[TX] = p[TX].coerceIn(-0.26, 0.26); p[TY] = p[TY].coerceIn(-0.26, 0.26)          // tilt within 15 degrees
        return p
    }

    /** The model at one parameter vector, with everything precomputed for fast distance queries. */
    class Model(params: DoubleArray, val zs: Double) {
        val p = clamp(params)
        private val r00: Double; private val r01: Double; private val r02: Double
        private val r10: Double; private val r11: Double; private val r12: Double
        private val r20: Double; private val r21: Double; private val r22: Double
        val cx: Double; val cy: Double; val cz: Double
        private val w = p[W]; private val l = p[L]; private val h = p[H]; private val rc = p[RC]; private val rz = p[RZ]; private val rb = p[RB]

        init {
            val cz_ = cos(p[TH]); val sz_ = sin(p[TH]); val cy_ = cos(p[TY]); val sy_ = sin(p[TY]); val cx_ = cos(p[TX]); val sx_ = sin(p[TX])
            // world-from-local rotation Rz(theta) Ry(ty) Rx(tx)
            r00 = cz_ * cy_; r01 = cz_ * sy_ * sx_ - sz_ * cx_; r02 = cz_ * sy_ * cx_ + sz_ * sx_
            r10 = sz_ * cy_; r11 = sz_ * sy_ * sx_ + cz_ * cx_; r12 = sz_ * sy_ * cx_ - cz_ * sx_
            r20 = -sy_; r21 = cy_ * sx_; r22 = cy_ * cx_
            // resting: the lowest point of the tilted body touches the sheet
            val rest = (w / 2 - rc) * abs(r20) + (l / 2 - rc) * abs(r21) + (rc - rb) * hypot(r20, r21) + rb * (1 - r22)
            cx = p[CX]; cy = p[CY]; cz = zs + rest
        }

        /** Tilt of the body's axis from the z axis, degrees. */
        val tiltDeg: Double get() = Math.toDegrees(Math.acos(r22.coerceIn(-1.0, 1.0)))

        /** Angle (degrees) between the body's axis and a given direction (nx, ny, nz), e.g. the normal of the sheet under it. */
        fun tiltFrom(nx: Double, ny: Double, nz: Double): Double {
            val n = sqrt(nx * nx + ny * ny + nz * nz)
            return Math.toDegrees(Math.acos(((r02 * nx + r12 * ny + r22 * nz) / n).coerceIn(-1.0, 1.0)))
        }

        /** Centre of the body (for bounding the ray search) and its bounding radius. */
        val mx: Double get() = cx + r02 * h / 2
        val my: Double get() = cy + r12 * h / 2
        val mz: Double get() = cz + r22 * h / 2
        val radius: Double get() = 0.5 * sqrt(w * w + l * l + h * h) + 3.0

        /** Signed distance (mm, negative inside) of a world point. */
        fun sdf(x: Double, y: Double, z: Double): Double {
            val ex = x - cx; val ey = y - cy; val ez = z - cz
            val lx = r00 * ex + r10 * ey + r20 * ez; val ly = r01 * ex + r11 * ey + r21 * ez; val lz = r02 * ex + r12 * ey + r22 * ez
            return local(lx, ly, lz)
        }

        /** Rounded box in its own frame: intersection of (rounded top, open below) and (rounded bottom, open above). */
        fun local(lx: Double, ly: Double, lz: Double): Double {
            val qx = abs(lx) - (w / 2 - rc); val qy = abs(ly) - (l / 2 - rc)
            val dfp = hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0) - rc          // distance to the footprint outline
            val top = cap(dfp + rz, lz - (h - rz), rz)
            val bottom = cap(dfp + rb, rb - lz, rb)
            return max(top, bottom)
        }

        private fun cap(d2: Double, dz: Double, r: Double) = min(max(d2, dz), 0.0) + hypot(max(d2, 0.0), max(dz, 0.0)) - r
    }

    /**
     * @param within fraction of the confirmed-edge rays within [TOL_MM] of the model: how box-like the object is
     * @param sigma  1-sigma repeatability of each parameter from leave-one-view-out refits (null if not computed)
     */
    class Result(val p: DoubleArray, val zs: Double, val rays: Int, val edges: Int, val views: Int, val within: Double, val medianAbs: Double,
                 val stickerResiduals: DoubleArray, val sigma: DoubleArray?) {
        val tiltDeg: Double get() = Model(p, zs).tiltDeg
    }

    private class Eval(val r: DoubleArray, val t: DoubleArray, val w: DoubleArray, val rs: DoubleArray, val ws: DoubleArray, val cost: Double)

    private val GR = (sqrt(5.0) - 1) / 2

    /** Minimum of the signed distance along ray i (convex along a line: golden section inside the bounding sphere). */
    private fun rayMin(m: Model, rays: Rays, i: Int, out: DoubleArray) {
        val ox = rays.ox[i]; val oy = rays.oy[i]; val oz = rays.oz[i]; val dx = rays.dx[i]; val dy = rays.dy[i]; val dz = rays.dz[i]
        val tc = (m.mx - ox) * dx + (m.my - oy) * dy + (m.mz - oz) * dz
        val rad = m.radius
        var a = max(0.0, tc - rad); var b = min(rays.t1[i], tc + rad)
        if (b < a) { val t = min(rays.t1[i], max(0.0, tc)); out[0] = m.sdf(ox + t * dx, oy + t * dy, oz + t * dz); out[1] = t; return }
        var c = b - GR * (b - a); var d = a + GR * (b - a)
        var fc = m.sdf(ox + c * dx, oy + c * dy, oz + c * dz); var fd = m.sdf(ox + d * dx, oy + d * dy, oz + d * dz)
        for (it in 0 until 24) {
            if (fc < fd) { b = d; d = c; fd = fc; c = b - GR * (b - a); fc = m.sdf(ox + c * dx, oy + c * dy, oz + c * dz) }
            else { a = c; c = d; fc = fd; d = a + GR * (b - a); fd = m.sdf(ox + d * dx, oy + d * dy, oz + d * dz) }
        }
        val t = (a + b) / 2
        out[0] = m.sdf(ox + t * dx, oy + t * dy, oz + t * dz); out[1] = t
    }

    private fun evaluate(p: DoubleArray, zs: Double, rays: Rays, stickers: DoubleArray, cOut: Double, cIn: Double, skipView: Int): Eval {
        val m = Model(p, zs); val n = rays.n
        val r = DoubleArray(n); val t = DoubleArray(n); val w = DoubleArray(n)
        val chunks = min(8, max(1, n / 256))
        Parallel.forEach(chunks) { ci ->
            val out = DoubleArray(2)
            for (i in (ci * n / chunks) until ((ci + 1) * n / chunks)) {
                if (rays.view[i] == skipView) continue
                rayMin(m, rays, i, out); r[i] = out[0]; t[i] = out[1]
            }
        }
        // Cauchy loss c^2 ln(1 + (x/c)^2) and its reweighting factor 1 / (1 + (x/c)^2): the normal equations built
        // from these weights follow exactly the gradient of the cost the steps are judged by
        var cost = 0.0
        for (i in 0 until n) {
            if (rays.view[i] == skipView) continue
            val x = r[i]
            val c = if (x > 0) cOut else cIn
            val u = (x / c) * (x / c)
            var wi = 1.0 / (1.0 + u); var li = c * c * ln(1.0 + u)
            if (!rays.edge[i] && x >= 0) { wi *= UNCONFIRMED; li *= UNCONFIRMED }          // only a bound: it counts when the model sticks out
            w[i] = wi; cost += li
        }
        val nS = stickers.size / 3
        val rs = DoubleArray(nS); val ws = DoubleArray(nS)
        val cs = max(STICKER_C, cOut)
        for (k in 0 until nS) {
            rs[k] = m.sdf(stickers[3 * k], stickers[3 * k + 1], stickers[3 * k + 2]) - STICKER_T
            val u = (rs[k] / cs) * (rs[k] / cs)
            ws[k] = STICKER_W / (1.0 + u); cost += STICKER_W * cs * cs * ln(1.0 + u)
        }
        val q = m.p; cost += RB_PRIOR * (q[RB] - q[RZ]) * (q[RB] - q[RZ])
        return Eval(r, t, w, rs, ws, cost)
    }

    /** Robust Levenberg-Marquardt at one robust scale. skipView >= 0 leaves that view out (jackknife). */
    private fun fitScale(start: DoubleArray, zs: Double, rays: Rays, stickers: DoubleArray, cOut: Double, iters: Int, skipView: Int = -1): Pair<DoubleArray, Eval> {
        val cIn = 2 * cOut; val n = rays.n; val nS = stickers.size / 3
        var p = clamp(start)
        var ev = evaluate(p, zs, rays, stickers, cOut, cIn, skipView)
        var lambda = 1e-2
        val A = Array(NP) { DoubleArray(NP) }; val g = DoubleArray(NP)
        val J = Array(NP) { DoubleArray(n) }; val Js = Array(NP) { DoubleArray(nS) }
        val px = DoubleArray(n); val py = DoubleArray(n); val pz = DoubleArray(n)
        for (iter in 0 until iters) {
            // envelope theorem: d(min_t f)/dp = df/dp at the minimiser, so the tangent points stay fixed for the Jacobian
            for (i in 0 until n) { val t = ev.t[i]; px[i] = rays.ox[i] + t * rays.dx[i]; py[i] = rays.oy[i] + t * rays.dy[i]; pz[i] = rays.oz[i] + t * rays.dz[i] }
            for (k in 0 until NP) {
                val q = p.copyOf(); q[k] += STEPS[k]; val mq = Model(q, zs)
                val Jk = J[k]
                for (i in 0 until n) Jk[i] = if (rays.view[i] == skipView) 0.0 else (mq.sdf(px[i], py[i], pz[i]) - ev.r[i]) / STEPS[k]
                for (s in 0 until nS) Js[k][s] = (mq.sdf(stickers[3 * s], stickers[3 * s + 1], stickers[3 * s + 2]) - STICKER_T - ev.rs[s]) / STEPS[k]
            }
            for (a in 0 until NP) {
                var ga = 0.0; val Ja = J[a]
                for (i in 0 until n) ga += ev.w[i] * Ja[i] * ev.r[i]
                for (s in 0 until nS) ga += ev.ws[s] * Js[a][s] * ev.rs[s]
                g[a] = ga
                for (b in a until NP) {
                    var sum = 0.0; val Jb = J[b]
                    for (i in 0 until n) sum += ev.w[i] * Ja[i] * Jb[i]
                    for (s in 0 until nS) sum += ev.ws[s] * Js[a][s] * Js[b][s]
                    A[a][b] = sum; A[b][a] = sum
                }
            }
            // prior row: (rb - rz) -> 0
            val dPrior = p[RB] - p[RZ]
            A[RB][RB] += RB_PRIOR; A[RZ][RZ] += RB_PRIOR; A[RB][RZ] -= RB_PRIOR; A[RZ][RB] -= RB_PRIOR
            g[RB] += RB_PRIOR * dPrior; g[RZ] -= RB_PRIOR * dPrior
            var improved = false; var rel = 0.0
            for (attempt in 0 until 8) {
                val Ad = Array(NP) { a -> DoubleArray(NP) { b -> A[a][b] + (if (a == b) lambda * (A[a][a] + 1e-9) else 0.0) } }
                val d = Linear.solve(Ad, DoubleArray(NP) { -g[it] }) ?: break
                // trust region: sizes change at most 10 % per step, yaw 8 deg, tilt 2 deg, centre 10 % of the size
                val big = max(p[W], p[L])
                var lim = 1.0
                lim = max(lim, abs(d[W]) / (0.10 * p[W])); lim = max(lim, abs(d[L]) / (0.10 * p[L])); lim = max(lim, abs(d[H]) / (0.10 * p[H]))
                lim = max(lim, abs(d[TH]) / Math.toRadians(8.0)); lim = max(lim, abs(d[TX]) / Math.toRadians(2.0)); lim = max(lim, abs(d[TY]) / Math.toRadians(2.0))
                lim = max(lim, abs(d[CX]) / (0.10 * big)); lim = max(lim, abs(d[CY]) / (0.10 * big))
                val cand = clamp(DoubleArray(NP) { p[it] + d[it] / lim })
                val e2 = evaluate(cand, zs, rays, stickers, cOut, cIn, skipView)
                if (e2.cost < ev.cost) {
                    rel = (ev.cost - e2.cost) / max(ev.cost, 1e-12)
                    p = cand; ev = e2; lambda = max(lambda / 3, 1e-7); improved = true
                    break
                }
                lambda *= 4
            }
            if (!improved || rel < 1e-6) break
        }
        return p to ev
    }

    /**
     * Fit from a starting guess (see [initFromPoints]). stickers: x, y, z triples of sticker centres in the z-up
     * frame. Returns null when there are too few rays. jackknife: also estimate the repeatability of every
     * parameter by refitting with each view left out.
     */
    fun fit(p0: DoubleArray, allRays: Rays, stickers: DoubleArray, zs: Double, jackknife: Boolean = true, log: (String) -> Unit = {}): Result? {
        val rays = allRays.thinned(MAX_RAYS)
        if (rays.n < 60) return null
        val base = clamp(p0)
        // a few starts at the coarsest scale (corner radius, yaw), then the best one goes down the schedule
        var best: Pair<DoubleArray, Eval>? = null
        val half = min(base[W], base[L]) / 2
        for ((frac, dyaw) in listOf(0.25 to 0.0, 0.6 to 0.0, 0.6 to Math.PI / 4)) {
            val s = base.copyOf(); s[RC] = frac * half; s[TH] = base[TH] + dyaw
            val r = fitScale(s, zs, rays, stickers, SCHEDULE[0], 30)
            if (best == null || r.second.cost < best.second.cost) best = r
        }
        var p = best!!.first; var ev = best.second
        for (si in 1 until SCHEDULE.size) { val r = fitScale(p, zs, rays, stickers, SCHEDULE[si], 30); p = r.first; ev = r.second }
        // statistics on the confirmed edges
        val res = ArrayList<Double>(); var within = 0; var nEdge = 0
        val viewSet = HashSet<Int>()
        for (i in 0 until rays.n) { viewSet.add(rays.view[i]); if (!rays.edge[i]) continue; nEdge++; val a = abs(ev.r[i]); res.add(a); if (a < TOL_MM) within++ }
        res.sort()
        val med = if (res.isEmpty()) Double.NaN else res[res.size / 2]
        log("ray fit: %d rays (%d confirmed edges) from %d views; %.0f%% of the edges within %.1f mm, median |r| %.3f mm".format(rays.n, nEdge, viewSet.size, if (nEdge > 0) 100.0 * within / nEdge else 0.0, TOL_MM, med))
        var sigma: DoubleArray? = null
        if (jackknife && viewSet.size >= 4) {
            val views = viewSet.sorted(); val est = ArrayList<DoubleArray>()
            for (v in views) est.add(fitScale(p, zs, rays, stickers, SCHEDULE.last(), 10, v).first)
            val mean = DoubleArray(NP); for (e in est) for (k in 0 until NP) mean[k] += e[k] / est.size
            val nv = est.size
            sigma = DoubleArray(NP) { k -> var s = 0.0; for (e in est) s += (e[k] - mean[k]) * (e[k] - mean[k]); sqrt(s * (nv - 1) / nv) }
        }
        return Result(p, zs, rays.n, nEdge, viewSet.size, if (nEdge > 0) within.toDouble() / nEdge else 0.0, med, ev.rs, sigma)
    }

    /**
     * Starting guess from a point cloud of the hull (x, y, z triples, z-up, base on z = 0): the minimum-area
     * bounding rectangle of the points above the base skirt, height from the top.
     */
    fun initFromPoints(verts: FloatArray, minZ: Double = 3.0): DoubleArray? {
        val n = verts.size / 3
        var count = 0; var hmax = 0.0
        for (i in 0 until n) if (verts[3 * i + 2] > minZ) { count++; hmax = max(hmax, verts[3 * i + 2].toDouble()) }
        if (count < 100) return null
        val stride = max(1, count / 4000)
        var best: DoubleArray? = null; var bestArea = Double.MAX_VALUE
        var deg = 0
        while (deg < 90) {
            val th = Math.toRadians(deg.toDouble()); val c = cos(th); val s = sin(th)
            var x0 = Double.MAX_VALUE; var x1 = -Double.MAX_VALUE; var y0 = Double.MAX_VALUE; var y1 = -Double.MAX_VALUE
            var q = 0
            for (i in 0 until n) {
                if (verts[3 * i + 2] <= minZ) continue
                if (q++ % stride != 0) continue
                val lx = c * verts[3 * i] + s * verts[3 * i + 1]; val ly = -s * verts[3 * i] + c * verts[3 * i + 1]      // rotate by -theta
                x0 = min(x0, lx); x1 = max(x1, lx); y0 = min(y0, ly); y1 = max(y1, ly)
            }
            val area = (x1 - x0) * (y1 - y0)
            if (area < bestArea) {
                bestArea = area
                val mx = (x0 + x1) / 2; val my = (y0 + y1) / 2
                val m = min(x1 - x0, y1 - y0)
                best = doubleArrayOf(c * mx - s * my, s * mx + c * my, th, x1 - x0, y1 - y0, hmax, 0.25 * m / 2, 0.12 * hmax, 0.0, 0.0, 0.08 * hmax)
            }
            deg += 2
        }
        return best?.let { clamp(it) }
    }

    /** Canonical form for reporting: W is the longer side, yaw in (-90, 90] degrees. Shape is unchanged. */
    fun canonical(p: DoubleArray): DoubleArray {
        val q = clamp(p)
        if (q[L] > q[W]) {            // new local x = old local y, new local y = -old local x
            val t = q[W]; q[W] = q[L]; q[L] = t; q[TH] += Math.PI / 2
            val tx = q[TX]; q[TX] = q[TY]; q[TY] = -tx
        }
        while (q[TH] > Math.PI / 2) { q[TH] -= Math.PI; q[TX] = -q[TX]; q[TY] = -q[TY] }
        while (q[TH] <= -Math.PI / 2) { q[TH] += Math.PI; q[TX] = -q[TX]; q[TY] = -q[TY] }
        return q
    }

    /**
     * Exact mesh of the fitted body for the clean STL, untilted and resting flat on z = 0 (which is what the
     * physical object is). Every vertex lies on the model surface: rays from the body centre are bisected onto the
     * zero set, so flat faces are flat and fillets are true arcs; no voxel staircase.
     * @param stepDeg angular sampling (1.5 deg ~ 0.9 mm on a 35 mm object)
     */
    fun mesh(params: DoubleArray, stepDeg: Double = 1.5): Mesh {
        val q = clamp(params); q[TX] = 0.0; q[TY] = 0.0
        val m = Model(q, 0.0)
        val h = q[H]
        val nAz = max(24, (360 / stepDeg).toInt()); val nEl = max(8, (180 / stepDeg).toInt() - 1)      // rings strictly between the poles
        val reach = hypot(q[W], q[L]) / 2 + h + 2.0
        val verts = FloatBuf(3 * (nAz * nEl + 2)); val tris = IntBuf(6 * nAz * nEl)
        fun add(x: Double, y: Double, z: Double): Int {
            val zz = if (abs(z) < 1e-6) 0.0 else if (abs(z - h) < 1e-6) h else z
            verts.add(x.toFloat()); verts.add(y.toFloat()); verts.add(zz.toFloat()); return verts.n / 3 - 1
        }
        val ring = Array(nEl) { IntArray(nAz) }
        val c0x = q[CX]; val c0y = q[CY]; val c0z = h / 2
        for (r in 0 until nEl) {
            val el = Math.toRadians(-90.0 + 180.0 * (r + 1) / (nEl + 1))
            for (a in 0 until nAz) {
                val az = 2 * Math.PI * a / nAz
                val dx = cos(el) * cos(az); val dy = cos(el) * sin(az); val dz = sin(el)
                var lo = 0.0; var hi = reach
                for (it in 0 until 44) { val mid = (lo + hi) / 2; if (m.sdf(c0x + mid * dx, c0y + mid * dy, c0z + mid * dz) <= 0) lo = mid else hi = mid }
                val t = (lo + hi) / 2
                ring[r][a] = add(c0x + t * dx, c0y + t * dy, c0z + t * dz)
            }
        }
        val top = add(c0x, c0y, h); val bottom = add(c0x, c0y, 0.0)
        for (r in 0 until nEl - 1) for (a in 0 until nAz) {
            val b = (a + 1) % nAz
            tris.add(ring[r][a]); tris.add(ring[r][b]); tris.add(ring[r + 1][a])
            tris.add(ring[r][b]); tris.add(ring[r + 1][b]); tris.add(ring[r + 1][a])
        }
        for (a in 0 until nAz) { val b = (a + 1) % nAz; tris.add(ring[nEl - 1][a]); tris.add(ring[nEl - 1][b]); tris.add(top) }
        for (a in 0 until nAz) { val b = (a + 1) % nAz; tris.add(ring[0][b]); tris.add(ring[0][a]); tris.add(bottom) }
        val mesh = Mesh(verts.toArray(), tris.toArray())
        return if (mesh.volume() < 0) mesh.flipped() else mesh
    }
}
