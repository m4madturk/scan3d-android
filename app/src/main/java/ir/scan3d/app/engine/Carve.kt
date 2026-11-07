package ir.scan3d.app.engine

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/** A binary silhouette (object = true) with the camera that produced it. */
class SilView(val w: Int, val h: Int, val mask: BooleanArray, val cam: Camera, val pose: Pose6) {
    /** Signed distance to the silhouette edge in pixels: positive inside. Two-pass EDT (Felzenszwalb). */
    fun signedDistance(): FloatArray {
        val din = edt(mask, false); val dout = edt(mask, true)
        return FloatArray(w * h) { sqrt(din[it]) - sqrt(dout[it]) }
    }

    private fun edt(m: BooleanArray, invert: Boolean): FloatArray {
        val inf = 1e12f
        val f = FloatArray(w * h) { if (m[it] != invert) inf else 0f }   // distance from pixels where (m xor invert) is false
        // columns
        val col = FloatArray(h); val d = FloatArray(h)
        for (x in 0 until w) { for (y in 0 until h) col[y] = f[y * w + x]; dt1d(col, d, h); for (y in 0 until h) f[y * w + x] = d[y] }
        val row = FloatArray(w); val dr = FloatArray(w)
        for (y in 0 until h) { for (x in 0 until w) row[x] = f[y * w + x]; dt1d(row, dr, w); for (x in 0 until w) f[y * w + x] = dr[x] }
        return f
    }

    private fun dt1d(f: FloatArray, d: FloatArray, n: Int) {
        val v = IntArray(n); val z = FloatArray(n + 1)
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

/**
 * Visual hull with signed-distance fusion (port of hull.py). Grid in the z-up frame; the board
 * plane is z = 0 and the object sits above it. Returns an indexed triangle mesh.
 */
object Carve {
    class Grid(val x0: Double, val y0: Double, val z0: Double, val nx: Int, val ny: Int, val nz: Int, val voxel: Double) {
        val f = FloatArray(nx * ny * nz)
        fun idx(i: Int, j: Int, k: Int) = (i * ny + j) * nz + k
    }

    fun carve(views: List<SilView>, bounds: Array<DoubleArray>, voxel: Double, marginPx: Double, planes: List<PlaneCut>, minVotesOut: Int,
              progress: (Double) -> Unit = {}): Grid {
        val nx = ((bounds[0][1] - bounds[0][0]) / voxel).toInt() + 1
        val ny = ((bounds[1][1] - bounds[1][0]) / voxel).toInt() + 1
        val nz = ((bounds[2][1] - bounds[2][0]) / voxel).toInt() + 1
        val g = Grid(bounds[0][0], bounds[1][0], bounds[2][0] + voxel / 2, nx, ny, nz, voxel)
        val sdfs = views.map { it.signedDistance() }
        val Rs = views.map { it.pose.R() }
        val votes = ShortArray(nx * ny * nz)
        java.util.Arrays.fill(g.f, Float.POSITIVE_INFINITY)
        for (i in 0 until nx) {
            val x = g.x0 + i * voxel
            for (j in 0 until ny) { val y = g.y0 + j * voxel
                for (k in 0 until nz) { val z = g.z0 + k * voxel
                    val id = g.idx(i, j, k)
                    var fmin = Float.POSITIVE_INFINITY; var nOut = 0
                    for ((vi, v) in views.withIndex()) {
                        val R = Rs[vi]; val t = v.pose.t
                        // board frame has z into the board: world point (x, y, -z)
                        val pcx = R[0][0] * x + R[0][1] * y - R[0][2] * z + t[0]
                        val pcy = R[1][0] * x + R[1][1] * y - R[1][2] * z + t[1]
                        val pcz = R[2][0] * x + R[2][1] * y - R[2][2] * z + t[2]
                        if (pcz <= 0) continue
                        val pr = v.cam.project(Vec3(pcx, pcy, pcz)) ?: continue
                        val u = pr[0]; val vv = pr[1]
                        if (u < 0 || u >= v.w - 1 || vv < 0 || vv >= v.h - 1) continue
                        val dpx = bilinear(sdfs[vi], v.w, u, vv) + marginPx
                        if (dpx < 0) nOut++
                        val dmm = (dpx * pcz / v.cam.f).toFloat()
                        if (dmm < fmin) fmin = dmm
                    }
                    if (minVotesOut > 1 && nOut < minVotesOut && fmin < 0) fmin = (voxel * 0.5).toFloat()
                    for (pl in planes) {
                        val rx = x - pl.p.x; val ry = y - pl.p.y; val rz = z - pl.p.z
                        val along = rx * pl.n.x + ry * pl.n.y + rz * pl.n.z
                        var dpl = -along
                        if (pl.radius != null) { val lx = rx - along * pl.n.x; val ly = ry - along * pl.n.y; val lz = rz - along * pl.n.z
                            if (sqrt(lx * lx + ly * ly + lz * lz) > pl.radius) dpl = Double.POSITIVE_INFINITY }
                        if (dpl < fmin) fmin = dpl.toFloat()
                    }
                    g.f[id] = if (fmin.isInfinite()) (-voxel).toFloat() else fmin
                    votes[id] = nOut.toShort()
                } }
            if (i % 8 == 0) progress(i.toDouble() / nx)
        }
        return g
    }

    private fun bilinear(img: FloatArray, w: Int, x: Double, y: Double): Double {
        val x0 = x.toInt(); val y0 = y.toInt(); val fx = x - x0; val fy = y - y0
        val o = y0 * w + x0
        return (img[o] * (1 - fx) + img[o + 1] * fx) * (1 - fy) + (img[o + w] * (1 - fx) + img[o + w + 1] * fx) * fy
    }

    /** Bounds (z-up, mm) of the negative region of a coarse grid, padded. */
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
