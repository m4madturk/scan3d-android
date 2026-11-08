package ir.scan3d.app.engine

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Fits a parametric "rounded box" to the scanned hull: footprint W x L with vertical-edge radius rc,
 * height H with top-edge radius rz, flat bottom resting on the board, yaw theta, plus a small tilt
 * (tx, ty) of the whole body about its base centre (a bent sheet under the object shows up as such a
 * tilt). Many everyday objects (cases, boxes, housings) are this shape; when the fit is tight the
 * parametric model is the clean, printable version of the scan, and its parameters are the
 * measurement the user actually wants (W, L, H, radii).
 *
 * The loss is robust and one-sided: the visual hull is a superset of the object, so hull material
 * outside the box (residual > 0) is down-weighted quickly (bloat near the base, missing low views),
 * while the box poking out of the hull (residual < 0) is penalised harder.
 */
object ShapeFit {
    class Params(var cx: Double, var cy: Double, var theta: Double, var w: Double, var l: Double, var h: Double,
                 var rc: Double, var rz: Double, var tx: Double = 0.0, var ty: Double = 0.0) {
        fun copy() = Params(cx, cy, theta, w, l, h, rc, rz, tx, ty)
        fun toArray() = doubleArrayOf(cx, cy, theta, w, l, h, rc, rz, tx, ty)
        /** Tilt of the body relative to the board plane, degrees. */
        val tiltDeg get() = Math.toDegrees(hypot(tx, ty))
        companion object {
            const val N = 10
            fun of(a: DoubleArray) = Params(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9])
        }
    }

    /**
     * @param rms       rms of all residuals (mm)
     * @param maxDev    largest |residual| (mm)
     * @param within1mm fraction of the hull surface within 1 mm of the fitted box: the "how box-like" score
     * @param outside   rms of the negative residuals only, i.e. how far the box pokes out of the hull (mm)
     */
    class Result(val params: Params, val rms: Double, val maxDev: Double, val samples: Int, val within1mm: Double, val outside: Double)

    private const val C_OUT = 1.0     // mm: hull bloat beyond this contributes little
    private const val C_IN = 2.0      // mm: dents in the hull (AI erosion, spikes) are tolerated too, but less
    private const val Z_MIN = 3.0     // mm: the base skirt is never trusted

    fun clamp(a: DoubleArray): DoubleArray {
        val p = a.copyOf()
        p[3] = max(p[3], 2.0); p[4] = max(p[4], 2.0); p[5] = max(p[5], 1.0)
        p[7] = p[7].coerceIn(0.05, min(p[5], min(p[3], p[4]) / 2) - 0.05)
        p[6] = p[6].coerceIn(p[7], min(p[3], p[4]) / 2 - 0.05)
        p[8] = p[8].coerceIn(-0.26, 0.26); p[9] = p[9].coerceIn(-0.26, 0.26)     // tilt within 15 degrees
        return p
    }

    /** Signed distance of point (x, y, z) to the rounded box; negative inside. Bottom is cut flat at the base plane. */
    fun sdf(p: Params, x: Double, y: Double, z: Double): Double {
        val c = cos(-p.theta); val s = sin(-p.theta)
        val dx = x - p.cx; val dy = y - p.cy
        var lx = c * dx - s * dy; var ly = s * dx + c * dy; var lz = z
        if (p.tx != 0.0 || p.ty != 0.0) {       // undo the body tilt about the base centre: local y-rotation by ty, then x-rotation by tx
            val cy = cos(-p.ty); val sy = sin(-p.ty); val x1 = cy * lx + sy * lz; val z1 = -sy * lx + cy * lz; lx = x1; lz = z1
            val cx = cos(-p.tx); val sx = sin(-p.tx); val y2 = cx * ly - sx * lz; val z2 = sx * ly + cx * lz; ly = y2; lz = z2
        }
        val hx = p.w / 2 - p.rz; val hy = p.l / 2 - p.rz; val r2 = p.rc - p.rz        // 2D rounded rectangle, shrunk by rz for the edge fillet
        val qx = abs(lx) - (hx - r2); val qy = abs(ly) - (hy - r2)
        val d2 = hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0) - r2
        val hz = (p.h + p.rz) / 2; val zc = lz - (p.h - hz)                            // rounded extrusion with its top at z = h
        val dz = abs(zc) - (hz - p.rz)
        val body = min(max(d2, dz), 0.0) + hypot(max(d2, 0.0), max(dz, 0.0)) - p.rz
        return max(body, -lz)                                                          // flat cut at the base plane
    }

    /** Initial guess from the footprint: minimum-area bounding rectangle over rotations, height from the top. */
    private fun initial(pts: List<DoubleArray>): DoubleArray {
        var best: DoubleArray? = null; var bestArea = Double.MAX_VALUE
        var hmax = 0.0; for (q in pts) hmax = max(hmax, q[2])
        var step = 0
        while (step < 90) {
            val th = Math.toRadians(step.toDouble()); val c = cos(-th); val s = sin(-th)
            var x0 = Double.MAX_VALUE; var x1 = -Double.MAX_VALUE; var y0 = Double.MAX_VALUE; var y1 = -Double.MAX_VALUE
            for (q in pts) { val lx = c * q[0] - s * q[1]; val ly = s * q[0] + c * q[1]; x0 = min(x0, lx); x1 = max(x1, lx); y0 = min(y0, ly); y1 = max(y1, ly) }
            val area = (x1 - x0) * (y1 - y0)
            if (area < bestArea) {
                bestArea = area
                val mx = (x0 + x1) / 2; val my = (y0 + y1) / 2; val cc = cos(th); val ss = sin(th)
                best = doubleArrayOf(cc * mx - ss * my, ss * mx + cc * my, th, x1 - x0, y1 - y0, hmax, 0.0, 0.0, 0.0, 0.0)
            }
            step += 2
        }
        return best!!
    }

    private fun weight(r: Double) = if (r > 0) 1.0 / (1.0 + (r / C_OUT) * (r / C_OUT)) else 1.0 / (1.0 + (r / C_IN) * (r / C_IN))

    private class LmResult(val p: DoubleArray, val cost: Double, val r: DoubleArray)

    /** Robust Levenberg-Marquardt (IRLS weights, numeric Jacobian, trust-region step limits). */
    private fun lm(start: DoubleArray, pts: List<DoubleArray>, iters: Int): LmResult {
        val n = pts.size; val m = Params.N
        val steps = doubleArrayOf(0.05, 0.05, Math.toRadians(0.2), 0.05, 0.05, 0.05, 0.05, 0.05, Math.toRadians(0.2), Math.toRadians(0.2))
        fun residuals(a: DoubleArray): DoubleArray { val q = Params.of(a); return DoubleArray(n) { sdf(q, pts[it][0], pts[it][1], pts[it][2]) } }
        fun wcost(r: DoubleArray): Double { var s = 0.0; for (x in r) s += weight(x) * x * x; return s }
        var p = clamp(start)
        var r = residuals(p); var cost = wcost(r)
        var lambda = 1e-2
        val J = Array(n) { DoubleArray(m) }
        val A = Array(m) { DoubleArray(m) }; val g = DoubleArray(m)
        for (iter in 0 until iters) {
            for (k in 0 until m) { val a2 = p.copyOf(); a2[k] += steps[k]; val r2 = residuals(a2); for (i in 0 until n) J[i][k] = (r2[i] - r[i]) / steps[k] }
            for (a in 0 until m) { g[a] = 0.0; for (b in 0 until m) A[a][b] = 0.0 }
            for (i in 0 until n) { val w = weight(r[i]); val ji = J[i]; for (a in 0 until m) { g[a] += w * ji[a] * r[i]; for (b in a until m) A[a][b] += w * ji[a] * ji[b] } }
            for (a in 0 until m) for (b in 0 until a) A[a][b] = A[b][a]
            var improved = false
            for (attempt in 0 until 8) {
                val Ad = Array(m) { a -> DoubleArray(m) { b -> A[a][b] + (if (a == b) lambda * (A[a][a] + 1e-9) else 0.0) } }
                val d = Linear.solve(Ad, DoubleArray(m) { -g[it] }) ?: break
                // trust region: dims change at most 15 % per step, yaw at most 10 deg, tilt 3 deg, centre 15 % of the size
                val big = max(p[3], p[4])
                val lim = listOf(abs(d[3]) / (0.15 * p[3]), abs(d[4]) / (0.15 * p[4]), abs(d[5]) / (0.15 * p[5]), abs(d[2]) / Math.toRadians(10.0),
                    abs(d[8]) / Math.toRadians(3.0), abs(d[9]) / Math.toRadians(3.0), abs(d[0]) / (0.15 * big), abs(d[1]) / (0.15 * big), 1.0).max()
                val cand = clamp(DoubleArray(m) { p[it] + d[it] / lim })
                val r2 = residuals(cand); val c2 = wcost(r2)
                if (c2 < cost) {
                    val rel = (cost - c2) / cost
                    p = cand; r = r2; cost = c2; lambda = max(lambda / 3, 1e-6); improved = true
                    if (rel < 1e-5) return LmResult(p, cost, r)
                    break
                }
                lambda *= 4
            }
            if (!improved) break
        }
        return LmResult(p, cost, r)
    }

    /**
     * Fit the hull mesh (mm, base on z = 0). Surface points below Z_MIN are ignored (base skirt).
     * Several starts (corner/edge radii) guard against local minima. Returns null if there is too little surface.
     */
    fun fit(mesh: Mesh, maxSamples: Int = 3000, iters: Int = 40): Result? {
        val all = ArrayList<DoubleArray>()
        for (i in 0 until mesh.vertexCount) { val z = mesh.verts[3 * i + 2].toDouble(); if (z > Z_MIN) all.add(doubleArrayOf(mesh.verts[3 * i].toDouble(), mesh.verts[3 * i + 1].toDouble(), z)) }
        if (all.size < 200) return null
        val stride = max(1, all.size / maxSamples)
        val pts = all.filterIndexed { i, _ -> i % stride == 0 }
        val p0 = initial(all)
        var best: LmResult? = null
        for (fc in doubleArrayOf(0.05, 0.15, 0.3)) for (fz in doubleArrayOf(0.05, 0.15, 0.3)) {
            val s = p0.copyOf(); s[6] = fc * min(s[3], s[4]); s[7] = fz * s[5]
            val res = lm(s, pts, iters)
            if (best == null || res.cost < best.cost) best = res
        }
        val b = best!!
        var ss = 0.0; var mx = 0.0; var near = 0; var neg = 0.0
        for (x in b.r) { ss += x * x; mx = max(mx, abs(x)); if (abs(x) < 1.0) near++; if (x < 0) neg += x * x }
        val prm = Params.of(b.p)
        if (prm.l > prm.w) {        // report W as the longer side, with theta along it (new local x = old local y, new y = -old x)
            val t = prm.w; prm.w = prm.l; prm.l = t; prm.theta += Math.PI / 2
            val tt = prm.tx; prm.tx = prm.ty; prm.ty = -tt
        }
        while (prm.theta > Math.PI / 2) { prm.theta -= Math.PI; prm.tx = -prm.tx; prm.ty = -prm.ty }     // half turn flips both local axes
        while (prm.theta <= -Math.PI / 2) { prm.theta += Math.PI; prm.tx = -prm.tx; prm.ty = -prm.ty }
        return Result(prm, sqrt(ss / b.r.size), mx, b.r.size, near.toDouble() / b.r.size, sqrt(neg / b.r.size))
    }

    /**
     * Exact mesh of the fitted shape for the clean STL: every vertex lies on the SDF zero set (found by
     * bisection along rays from the base centre), so flat faces are flat and fillets are true arcs; no voxel
     * staircase. The body is generated untilted and resting flat on z = 0, which is what the physical object is.
     * @param stepDeg angular sampling of the dome (1.5 deg ~ 0.9 mm on a 35 mm object)
     */
    fun mesh(p: Params, stepDeg: Double = 1.5): Mesh {
        val q = p.copy(); q.tx = 0.0; q.ty = 0.0
        val nAz = max(24, (360 / stepDeg).toInt()); val nEl = max(6, (90 / stepDeg).toInt())     // rings el = 0 (rim) .. 90 (pole)
        val reach = hypot(q.w, q.l) / 2 + q.h + 2.0
        val verts = ArrayList<Float>(3 * (nAz * nEl + 2)); val tris = ArrayList<Int>(6 * nAz * nEl)
        fun add(x: Double, y: Double, z: Double): Int { verts.add(x.toFloat()); verts.add(y.toFloat()); verts.add(z.toFloat()); return verts.size / 3 - 1 }
        // rings 0 .. nEl-1 (el = 0 .. just below 90), then the top pole
        val ring = Array(nEl) { IntArray(nAz) }
        for (r in 0 until nEl) {
            val el = Math.toRadians(90.0 * r / nEl)
            for (a in 0 until nAz) {
                val az = 2 * Math.PI * a / nAz
                val dx = cos(el) * cos(az); val dy = cos(el) * sin(az); val dz = sin(el)
                var lo = 0.0; var hi = reach
                for (it in 0 until 40) { val mid = (lo + hi) / 2; if (sdf(q, q.cx + mid * dx, q.cy + mid * dy, mid * dz) <= 0) lo = mid else hi = mid }
                val t = (lo + hi) / 2
                ring[r][a] = add(q.cx + t * dx, q.cy + t * dy, if (r == 0) 0.0 else t * dz)
            }
        }
        val top = add(q.cx, q.cy, q.h); val bottom = add(q.cx, q.cy, 0.0)
        for (r in 0 until nEl - 1) for (a in 0 until nAz) {
            val b = (a + 1) % nAz
            tris.add(ring[r][a]); tris.add(ring[r][b]); tris.add(ring[r + 1][a])
            tris.add(ring[r][b]); tris.add(ring[r + 1][b]); tris.add(ring[r + 1][a])
        }
        for (a in 0 until nAz) { val b = (a + 1) % nAz; tris.add(ring[nEl - 1][a]); tris.add(ring[nEl - 1][b]); tris.add(top) }
        for (a in 0 until nAz) { val b = (a + 1) % nAz; tris.add(ring[0][b]); tris.add(ring[0][a]); tris.add(bottom) }
        val m = Mesh(verts.toFloatArray(), tris.toIntArray())
        return if (m.volume() < 0) Mesh(m.verts, IntArray(m.tris.size) { val t = it / 3; val w = it % 3; m.tris[3 * t + intArrayOf(0, 2, 1)[w]] }) else m
    }
}
