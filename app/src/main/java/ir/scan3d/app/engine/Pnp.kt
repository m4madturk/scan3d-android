package ir.scan3d.app.engine

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Pose from 2D-3D correspondences, pure Kotlin:
 *   1. planar homography DLT (all our initial points lie on the board plane z = 0)
 *   2. decomposition into R, t
 *   3. Gauss-Newton refinement on reprojection error with the full camera model
 * Also a free-point refinement used by the solver for video frames (points not coplanar).
 */
object Pnp {
    /** obj: world points (z may be 0 for board points), img: pixels. Returns pose or null. */
    fun solve(obj: List<Vec3>, img: List<DoubleArray>, cam: Camera, init: Pose6? = null): Pose6? {
        if (obj.size < 4) return null
        var pose = init ?: planarInit(obj, img, cam) ?: return null
        pose = refine(obj, img, cam, pose, 30)
        return pose
    }

    private fun planarInit(obj: List<Vec3>, img: List<DoubleArray>, cam: Camera): Pose6? {
        // normalised, undistorted image coordinates
        val n = obj.size
        val A = Array(2 * n) { DoubleArray(9) }
        for (i in 0 until n) {
            val (xn, yn) = cam.undistort(img[i][0], img[i][1]).let { it[0] to it[1] }
            val X = obj[i].x; val Y = obj[i].y
            A[2 * i] = doubleArrayOf(-X, -Y, -1.0, 0.0, 0.0, 0.0, xn * X, xn * Y, xn)
            A[2 * i + 1] = doubleArrayOf(0.0, 0.0, 0.0, -X, -Y, -1.0, yn * X, yn * Y, yn)
        }
        val h = nullVector(A) ?: return null
        // H = [h1 h2 h3] maps (X, Y, 1) -> normalised; columns r1, r2, t up to scale
        val h1 = Vec3(h[0], h[3], h[6]); val h2 = Vec3(h[1], h[4], h[7]); val h3 = Vec3(h[2], h[5], h[8])
        var s = 2.0 / (h1.norm() + h2.norm())
        if (h3.z * s < 0) s = -s                       // board must be in front of the camera
        val r1 = h1 * s; val r2 = h2 * s; val t = h3 * s
        // orthonormalise r1, r2
        val r1u = r1.unit(); val r2o = (r2 - r1u * r2.dot(r1u)).unit(); val r3 = r1u.cross(r2o)
        val R = arrayOf(doubleArrayOf(r1u.x, r2o.x, r3.x), doubleArrayOf(r1u.y, r2o.y, r3.y), doubleArrayOf(r1u.z, r2o.z, r3.z))
        return Pose6(Rot.toVec(R), t.toArray())
    }

    /** Smallest right singular vector of A via power iteration on (A^T A)^-1 (shifted inverse iteration). */
    private fun nullVector(A: Array<DoubleArray>): DoubleArray? {
        val n = 9
        val M = Array(n) { DoubleArray(n) }
        for (row in A) for (i in 0 until n) for (j in 0 until n) M[i][j] += row[i] * row[j]
        // shift slightly to keep it positive definite, then inverse iteration
        var tr = 0.0; for (i in 0 until n) tr += M[i][i]
        val eps = tr * 1e-10 + 1e-12
        for (i in 0 until n) M[i][i] += eps
        var v = DoubleArray(n) { 1.0 / sqrt(n.toDouble()) }
        for (it in 0 until 60) {
            val w = Linear.choleskySolve(M, v) ?: return null
            var nn = 0.0; for (x in w) nn += x * x
            nn = sqrt(nn); if (nn == 0.0) return null
            for (i in 0 until n) v[i] = w[i] / nn
        }
        return v
    }

    fun refine(obj: List<Vec3>, img: List<DoubleArray>, cam: Camera, start: Pose6, iters: Int): Pose6 {
        var p = start.copy()
        var lambda = 1e-3
        var cost = cost(obj, img, cam, p)
        for (it in 0 until iters) {
            val JtJ = Array(6) { DoubleArray(6) }; val Jtr = DoubleArray(6)
            val n = obj.size
            for (i in 0 until n) {
                val r0 = residual(obj[i], img[i], cam, p) ?: continue
                val J = Array(2) { DoubleArray(6) }
                for (k in 0 until 6) {
                    val d = 1e-6
                    val q = p.copy(); if (k < 3) q.r[k] += d else q.t[k - 3] += d
                    val r1 = residual(obj[i], img[i], cam, q) ?: continue
                    J[0][k] = (r1[0] - r0[0]) / d; J[1][k] = (r1[1] - r0[1]) / d
                }
                for (a in 0 until 6) { for (b in 0 until 6) JtJ[a][b] += J[0][a] * J[0][b] + J[1][a] * J[1][b]; Jtr[a] += J[0][a] * r0[0] + J[1][a] * r0[1] }
            }
            var improved = false
            for (tries in 0 until 6) {
                val Aug = Array(6) { i -> DoubleArray(6) { j -> JtJ[i][j] + (if (i == j) lambda * (1 + JtJ[i][i]) else 0.0) } }
                val dx = Linear.choleskySolve(Aug, DoubleArray(6) { -Jtr[it] }) ?: break
                val q = p.copy(); for (k in 0 until 3) { q.r[k] += dx[k]; q.t[k] += dx[k + 3] }
                val c2 = cost(obj, img, cam, q)
                if (c2 < cost) { p = q; cost = c2; lambda = maxOf(lambda / 3, 1e-9); improved = true; break } else lambda *= 5
            }
            if (!improved) break
        }
        return p
    }

    private fun residual(o: Vec3, im: DoubleArray, cam: Camera, p: Pose6): DoubleArray? {
        val pr = cam.project(p.toCamera(o)) ?: return null
        return doubleArrayOf(pr[0] - im[0], pr[1] - im[1])
    }

    private fun cost(obj: List<Vec3>, img: List<DoubleArray>, cam: Camera, p: Pose6): Double {
        var c = 0.0
        for (i in obj.indices) { val r = residual(obj[i], img[i], cam, p) ?: return Double.MAX_VALUE; c += r[0] * r[0] + r[1] * r[1] }
        return c
    }

    fun rms(obj: List<Vec3>, img: List<DoubleArray>, cam: Camera, p: Pose6): Double =
        sqrt(cost(obj, img, cam, p) / maxOf(1, obj.size))
}
