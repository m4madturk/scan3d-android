package ir.scan3d.app.engine

import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/** A binary silhouette (object = true) with the camera that produced it. */
class SilView(val w: Int, val h: Int, val mask: BooleanArray, val cam: Camera, val pose: Pose6) {
    /** Signed distance to the silhouette edge in pixels, positive inside. Computed once (two-pass EDT, Felzenszwalb). */
    val sdf: FloatArray by lazy {
        val din = edt(mask, false); val dout = edt(mask, true)
        FloatArray(w * h) { sqrt(din[it]) - sqrt(dout[it]) }
    }

    fun signedDistance(): FloatArray = sdf

    private fun edt(m: BooleanArray, invert: Boolean): FloatArray {
        val inf = 1e12f
        val f = FloatArray(w * h) { if (m[it] != invert) inf else 0f }   // distance from pixels where (m xor invert) is false
        val n = max(w, h)
        val col = FloatArray(n); val d = FloatArray(n); val v = IntArray(n); val z = FloatArray(n + 1)
        for (x in 0 until w) { for (y in 0 until h) col[y] = f[y * w + x]; dt1d(col, d, h, v, z); for (y in 0 until h) f[y * w + x] = d[y] }
        for (y in 0 until h) { System.arraycopy(f, y * w, col, 0, w); dt1d(col, d, w, v, z); System.arraycopy(d, 0, f, y * w, w) }
        return f
    }

    private fun dt1d(f: FloatArray, d: FloatArray, n: Int, v: IntArray, z: FloatArray) {
        var k = 0; v[0] = 0; z[0] = -Float.MAX_VALUE; z[1] = Float.MAX_VALUE
        for (q in 1 until n) {
            var s = ((f[q] + q.toFloat() * q) - (f[v[k]] + v[k].toFloat() * v[k])) / (2f * q - 2f * v[k])
            while (s <= z[k]) { k--; s = ((f[q] + q.toFloat() * q) - (f[v[k]] + v[k].toFloat() * v[k])) / (2f * q - 2f * v[k]) }
            k++; v[k] = q; z[k] = s; z[k + 1] = Float.MAX_VALUE
        }
        k = 0
        for (q in 0 until n) { while (z[k + 1] < q) k++; val dq = q - v[k]; d[q] = dq.toFloat() * dq + f[v[k]] }
    }
}

class PlaneCut(val p: Vec3, val n: Vec3, val radius: Double?)   // z-up frame

/** Runs `body(index)` for every index in [0, n) on a small pool of threads; the first exception is rethrown. */
object Parallel {
    /** Leave one core for the UI and never take more than 6: the phone must not stall or cook. */
    val threads: Int get() = max(1, min(6, Runtime.getRuntime().availableProcessors() - 1))

    fun forEach(n: Int, maxThreads: Int = threads, body: (Int) -> Unit) {
        val t = max(1, min(maxThreads, n))
        if (t == 1) { for (i in 0 until n) body(i); return }
        val next = AtomicInteger(0); val error = AtomicReference<Throwable?>(null)
        val workers = List(t) {
            Thread {
                try { while (error.get() == null) { val i = next.getAndIncrement(); if (i >= n) break; body(i) } }
                catch (e: Throwable) { error.compareAndSet(null, e) }
            }.also { th -> th.priority = Thread.NORM_PRIORITY - 1; th.start() }
        }
        workers.forEach { it.join() }
        error.get()?.let { throw it }
    }
}

/**
 * Visual hull with signed-distance fusion. Grid in the z-up frame; the board plane is z = 0 and the object
 * sits above it. The inner loop is allocation-free and runs on several threads (one x-slab per task): the
 * previous version built a Vec3 and a DoubleArray per voxel and view on a single core.
 */
object Carve {
    class Grid(val x0: Double, val y0: Double, val z0: Double, val nx: Int, val ny: Int, val nz: Int, val voxel: Double) {
        val f = FloatArray(nx * ny * nz)
        fun idx(i: Int, j: Int, k: Int) = (i * ny + j) * nz + k
    }

    /** Everything the inner loop needs from one view, as plain numbers. */
    private class V(view: SilView) {
        val sdf = view.sdf; val w = view.w; val h = view.h
        val r = view.pose.R(); val t = view.pose.t
        val f = view.cam.f; val cx = view.cam.cx; val cy = view.cam.cy; val k1 = view.cam.k1; val k2 = view.cam.k2
    }

    fun carve(views: List<SilView>, bounds: Array<DoubleArray>, voxel: Double, marginPx: Double, planes: List<PlaneCut>, minVotesOut: Int,
              progress: (Double) -> Unit = {}): Grid {
        val nx = ((bounds[0][1] - bounds[0][0]) / voxel).toInt() + 1
        val ny = ((bounds[1][1] - bounds[1][0]) / voxel).toInt() + 1
        val nz = ((bounds[2][1] - bounds[2][0]) / voxel).toInt() + 1
        val g = Grid(bounds[0][0], bounds[1][0], bounds[2][0] + voxel / 2, nx, ny, nz, voxel)
        Parallel.forEach(views.size) { views[it].sdf }             // distance transforms, in parallel
        val vs = views.map { V(it) }
        val nV = vs.size
        // robust fusion: the voxel's distance is the kth most-outside value (k = minVotesOut), so up to
        // k-1 wrong silhouettes (shadow, hand, mis-segmented board) cannot carve away the object
        val kk = max(1, minVotesOut)
        val done = AtomicInteger(0); val lock = Any()
        Parallel.forEach(nx) { i ->
            val low = FloatArray(kk)
            val ax = DoubleArray(nV); val ay = DoubleArray(nV); val az = DoubleArray(nV)
            val x = g.x0 + i * voxel
            for (j in 0 until ny) {
                val y = g.y0 + j * voxel
                // camera coordinates are linear in z along a column: pc = a - z * (third column of R)
                for (vi in 0 until nV) { val v = vs[vi]; val r = v.r
                    ax[vi] = r[0][0] * x + r[0][1] * y + v.t[0]; ay[vi] = r[1][0] * x + r[1][1] * y + v.t[1]; az[vi] = r[2][0] * x + r[2][1] * y + v.t[2] }
                var id = g.idx(i, j, 0)
                for (k in 0 until nz) {
                    val z = g.z0 + k * voxel
                    var fmin = Float.POSITIVE_INFINITY
                    if (kk > 1) java.util.Arrays.fill(low, Float.POSITIVE_INFINITY)
                    for (vi in 0 until nV) {
                        val v = vs[vi]; val r = v.r
                        // board frame has z into the board: world point (x, y, -z)
                        val pcz = az[vi] - r[2][2] * z
                        if (pcz <= 1e-9) continue
                        val xn = (ax[vi] - r[0][2] * z) / pcz; val yn = (ay[vi] - r[1][2] * z) / pcz
                        val r2 = xn * xn + yn * yn; val d = 1 + v.k1 * r2 + v.k2 * r2 * r2
                        val u = v.f * xn * d + v.cx; val vv = v.f * yn * d + v.cy
                        if (u < 0 || u >= v.w - 1 || vv < 0 || vv >= v.h - 1) continue
                        val x0 = u.toInt(); val y0 = vv.toInt(); val fx = (u - x0).toFloat(); val fy = (vv - y0).toFloat()
                        val o = y0 * v.w + x0; val sd = v.sdf
                        val dpx = (sd[o] * (1 - fx) + sd[o + 1] * fx) * (1 - fy) + (sd[o + v.w] * (1 - fx) + sd[o + v.w + 1] * fx) * fy + marginPx
                        val dmm = (dpx * pcz / v.f).toFloat()
                        if (kk == 1) { if (dmm < fmin) fmin = dmm }
                        else if (dmm < low[kk - 1]) {               // insert into the sorted k smallest
                            var p = kk - 1; while (p > 0 && low[p - 1] > dmm) { low[p] = low[p - 1]; p-- }; low[p] = dmm
                        }
                    }
                    if (kk > 1) fmin = if (low[kk - 1].isInfinite()) { var m = Float.POSITIVE_INFINITY; for (q in 0 until kk) if (!low[q].isInfinite()) m = low[q]; m } else low[kk - 1]
                    for (pl in planes) {
                        val rx = x - pl.p.x; val ry = y - pl.p.y; val rz = z - pl.p.z
                        val along = rx * pl.n.x + ry * pl.n.y + rz * pl.n.z
                        var dpl = -along
                        if (pl.radius != null) { val lx = rx - along * pl.n.x; val ly = ry - along * pl.n.y; val lz = rz - along * pl.n.z
                            if (sqrt(lx * lx + ly * ly + lz * lz) > pl.radius) dpl = Double.POSITIVE_INFINITY }
                        if (dpl < fmin) fmin = dpl.toFloat()
                    }
                    g.f[id] = if (fmin.isInfinite()) (-voxel).toFloat() else fmin
                    id++
                }
            }
            val n = done.incrementAndGet()
            if (n % 8 == 0) synchronized(lock) { progress(n.toDouble() / nx) }
        }
        return g
    }

    /**
     * Removes the thin "skirt" that the object's shadow leaves on the board plane (all views see the shadow
     * as foreground, so carving keeps a 1-2 voxel plate around the base): any (x, y) column whose solid
     * part is lower than minHeightMm is cleared. Returns the number of cleared columns.
     */
    fun removeSkirt(g: Grid, minHeightMm: Double): Int {
        var cleared = 0
        for (i in 0 until g.nx) for (j in 0 until g.ny) {
            var top = -1
            for (k in g.nz - 1 downTo 0) if (g.f[g.idx(i, j, k)] > 0) { top = k; break }
            if (top < 0) continue
            if ((top + 1) * g.voxel < minHeightMm) { for (k in 0..top) g.f[g.idx(i, j, k)] = (-g.voxel * 0.5).toFloat(); cleared++ }
        }
        return cleared
    }

    /** Diagnostics: per view, which fraction of the sampled volume projects inside its silhouette. */
    fun insideFractions(views: List<SilView>, bounds: Array<DoubleArray>, step: Double): DoubleArray {
        val out = DoubleArray(views.size)
        for ((vi, v) in views.withIndex()) {
            val R = v.pose.R(); val t = v.pose.t; var n = 0; var ins = 0
            var x = bounds[0][0]; while (x <= bounds[0][1]) { var y = bounds[1][0]; while (y <= bounds[1][1]) { var z = bounds[2][0]; while (z <= bounds[2][1]) {
                n++
                val pcx = R[0][0] * x + R[0][1] * y - R[0][2] * z + t[0]; val pcy = R[1][0] * x + R[1][1] * y - R[1][2] * z + t[1]; val pcz = R[2][0] * x + R[2][1] * y - R[2][2] * z + t[2]
                if (pcz > 0) v.cam.project(Vec3(pcx, pcy, pcz))?.let { pr -> val u = pr[0].toInt(); val w = pr[1].toInt(); if (u in 0 until v.w && w in 0 until v.h && v.mask[w * v.w + u]) ins++ }
                z += step }; y += step }; x += step }
            out[vi] = if (n > 0) ins.toDouble() / n else 0.0
        }
        return out
    }

    /** Bounds (z-up, mm) of the inside region (f > 0) of a grid, padded; null when nothing is inside. */
    fun insideBounds(g: Grid, pad: Double): Array<DoubleArray>? {
        var x0 = Double.MAX_VALUE; var x1 = -Double.MAX_VALUE; var y0 = Double.MAX_VALUE; var y1 = -Double.MAX_VALUE; var z1 = -Double.MAX_VALUE
        for (i in 0 until g.nx) for (j in 0 until g.ny) for (k in 0 until g.nz) if (g.f[g.idx(i, j, k)] > 0) {
            val x = g.x0 + i * g.voxel; val y = g.y0 + j * g.voxel; val z = g.z0 + k * g.voxel
            x0 = min(x0, x); x1 = max(x1, x); y0 = min(y0, y); y1 = max(y1, y); z1 = max(z1, z)
        }
        if (x1 < x0) return null
        return arrayOf(doubleArrayOf(x0 - pad, x1 + pad), doubleArrayOf(y0 - pad, y1 + pad), doubleArrayOf(0.0, z1 + pad))
    }
}
