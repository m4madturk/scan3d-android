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
        // normalised, undistorted image coordinates; both point sets Hartley-normalised (centroid 0, mean radius sqrt 2)
        // so the 9x9 normal matrix is well conditioned on real data (mm-scale board vs unit-scale image)
        val n = obj.size
        val und = Array(n) { cam.undistort(img[it][0], img[it][1]) }
        var mx = 0.0; var my = 0.0; var ux = 0.0; var uy = 0.0
        for (i in 0 until n) { mx += obj[i].x; my += obj[i].y; ux += und[i][0]; uy += und[i][1] }
        mx /= n; my /= n; ux /= n; uy /= n
        var dObj = 0.0; var dImg = 0.0
        for (i in 0 until n) { dObj += Math.hypot(obj[i].x - mx, obj[i].y - my); dImg += Math.hypot(und[i][0] - ux, und[i][1] - uy) }
        val sObj = sqrt(2.0) / (dObj / n).coerceAtLeast(1e-9); val sImg = sqrt(2.0) / (dImg / n).coerceAtLeast(1e-9)
        val A = Array(2 * n) { DoubleArray(9) }
        for (i in 0 until n) {
            val X = (obj[i].x - mx) * sObj; val Y = (obj[i].y - my) * sObj
            val xn = (und[i][0] - ux) * sImg; val yn = (und[i][1] - uy) * sImg
            A[2 * i] = doubleArrayOf(-X, -Y, -1.0, 0.0, 0.0, 0.0, xn * X, xn * Y, xn)
            A[2 * i + 1] = doubleArrayOf(0.0, 0.0, 0.0, -X, -Y, -1.0, yn * X, yn * Y, yn)
        }
        val hn = nullVector(A) ?: return null
        // denormalise: H = Timg^-1 * Hn * Tobj
        val Hn = arrayOf(doubleArrayOf(hn[0], hn[1], hn[2]), doubleArrayOf(hn[3], hn[4], hn[5]), doubleArrayOf(hn[6], hn[7], hn[8]))
        val Tobj = arrayOf(doubleArrayOf(sObj, 0.0, -sObj * mx), doubleArrayOf(0.0, sObj, -sObj * my), doubleArrayOf(0.0, 0.0, 1.0))
        val TimgInv = arrayOf(doubleArrayOf(1 / sImg, 0.0, ux), doubleArrayOf(0.0, 1 / sImg, uy), doubleArrayOf(0.0, 0.0, 1.0))
        val H = mul(TimgInv, mul(Hn, Tobj))
        val h1 = Vec3(H[0][0], H[1][0], H[2][0]); val h2 = Vec3(H[0][1], H[1][1], H[2][1]); val h3 = Vec3(H[0][2], H[1][2], H[2][2])
        var s = 2.0 / (h1.norm() + h2.norm())
        if (h3.z * s < 0) s = -s                       // board must be in front of the camera
        val r1 = h1 * s; val r2 = h2 * s; val t = h3 * s
        // orthonormalise r1, r2
        val r1u = r1.unit(); val r2o = (r2 - r1u * r2.dot(r1u)).unit(); val r3 = r1u.cross(r2o)
        val R = arrayOf(doubleArrayOf(r1u.x, r2o.x, r3.x), doubleArrayOf(r1u.y, r2o.y, r3.y), doubleArrayOf(r1u.z, r2o.z, r3.z))
        return Pose6(Rot.toVec(R), t.toArray())
    }

    private fun mul(a: Array<DoubleArray>, b: Array<DoubleArray>): Array<DoubleArray> = Array(3) { i -> DoubleArray(3) { j -> a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] } }

    /** Eigenvector of the smallest eigenvalue of A^T A (cyclic Jacobi on the 9x9 symmetric matrix; always converges). */
    private fun nullVector(A: Array<DoubleArray>): DoubleArray? {
        val n = 9
        val M = Array(n) { DoubleArray(n) }
        for (row in A) for (i in 0 until n) for (j in 0 until n) M[i][j] += row[i] * row[j]
        val V = Array(n) { i -> DoubleArray(n) { j -> if (i == j) 1.0 else 0.0 } }
        for (sweep in 0 until 60) {
            var off = 0.0; for (i in 0 until n) for (j in i + 1 until n) off += M[i][j] * M[i][j]
            if (off < 1e-24) break
            for (p in 0 until n) for (q in p + 1 until n) {
                if (abs(M[p][q]) < 1e-300) continue
                val theta = (M[q][q] - M[p][p]) / (2 * M[p][q])
                val tt = (if (theta >= 0) 1.0 else -1.0) / (abs(theta) + sqrt(theta * theta + 1))
                val c = 1 / sqrt(tt * tt + 1); val sn = tt * c
                for (k in 0 until n) { val mkp = M[k][p]; val mkq = M[k][q]; M[k][p] = c * mkp - sn * mkq; M[k][q] = sn * mkp + c * mkq }
                for (k in 0 until n) { val mpk = M[p][k]; val mqk = M[q][k]; M[p][k] = c * mpk - sn * mqk; M[q][k] = sn * mpk + c * mqk }
                for (k in 0 until n) { val vkp = V[k][p]; val vkq = V[k][q]; V[k][p] = c * vkp - sn * vkq; V[k][q] = sn * vkp + c * vkq }
            }
        }
        var best = 0; for (i in 1 until n) if (M[i][i] < M[best][best]) best = i
        val v = DoubleArray(n) { V[it][best] }
        var nn = 0.0; for (x in v) nn += x * x; if (nn == 0.0 || nn.isNaN()) return null
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
