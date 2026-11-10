package ir.scan3d.app.engine

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

/** Minimal dense linear algebra and camera geometry, pure Kotlin (JVM-testable, no OpenCV). */

class Vec3(@JvmField var x: Double, @JvmField var y: Double, @JvmField var z: Double) {
    operator fun plus(o: Vec3) = Vec3(x + o.x, y + o.y, z + o.z)
    operator fun minus(o: Vec3) = Vec3(x - o.x, y - o.y, z - o.z)
    operator fun times(s: Double) = Vec3(x * s, y * s, z * s)
    operator fun unaryMinus() = Vec3(-x, -y, -z)
    fun dot(o: Vec3) = x * o.x + y * o.y + z * o.z
    fun cross(o: Vec3) = Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x)
    fun norm() = sqrt(x * x + y * y + z * z)
    fun unit(): Vec3 { val n = norm(); return if (n > 0) this * (1.0 / n) else Vec3(0.0, 0.0, 0.0) }
    fun toArray() = doubleArrayOf(x, y, z)
    override fun toString() = "(%.4f, %.4f, %.4f)".format(x, y, z)
    companion object { fun of(a: DoubleArray) = Vec3(a[0], a[1], a[2]) }
}

/** 3x3 rotation as row-major array. */
object Rot {
    /** Rodrigues: rotation vector -> matrix. */
    fun fromVec(r: DoubleArray): Array<DoubleArray> {
        val th = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2])
        if (th < 1e-12) return arrayOf(doubleArrayOf(1.0, 0.0, 0.0), doubleArrayOf(0.0, 1.0, 0.0), doubleArrayOf(0.0, 0.0, 1.0))
        val a = r[0] / th; val b = r[1] / th; val c = r[2] / th
        val co = cos(th); val si = sin(th); val v = 1 - co
        return arrayOf(
            doubleArrayOf(co + a * a * v, a * b * v - c * si, a * c * v + b * si),
            doubleArrayOf(b * a * v + c * si, co + b * b * v, b * c * v - a * si),
            doubleArrayOf(c * a * v - b * si, c * b * v + a * si, co + c * c * v))
    }

    /**
     * matrix -> rotation vector (inverse Rodrigues), valid for every angle. The antisymmetric part of R gives the
     * axis times 2 sin(angle); it vanishes at 180 degrees (a camera that sees the sheet exactly upside down), where
     * the axis is taken from the symmetric part instead. The previous version returned a zero vector there.
     */
    fun toVec(R: Array<DoubleArray>): DoubleArray {
        val ax = R[2][1] - R[1][2]; val ay = R[0][2] - R[2][0]; val az = R[1][0] - R[0][1]
        val s2 = sqrt(ax * ax + ay * ay + az * az)                                   // 2 sin(angle)
        val c = ((R[0][0] + R[1][1] + R[2][2] - 1) / 2).coerceIn(-1.0, 1.0)          // cos(angle)
        val th = Math.atan2(s2 / 2, c)
        if (th < 1e-12) return doubleArrayOf(0.0, 0.0, 0.0)
        if (s2 > 1e-4) return doubleArrayOf(ax * th / s2, ay * th / s2, az * th / s2)
        if (c > 0) return doubleArrayOf(ax / 2, ay / 2, az / 2)                       // tiny angle: sin(angle) = angle
        // angle = pi (to 1e-4): R + R^T = 2 a a^T - 2 cos(angle) I ... solve for the axis from the largest diagonal term
        val xx = (R[0][0] - c) / (1 - c); val yy = (R[1][1] - c) / (1 - c); val zz = (R[2][2] - c) / (1 - c)
        val a = DoubleArray(3)
        if (xx >= yy && xx >= zz) { a[0] = sqrt(max(xx, 0.0)); a[1] = (R[0][1] + R[1][0]) / (2 * (1 - c) * a[0]); a[2] = (R[0][2] + R[2][0]) / (2 * (1 - c) * a[0]) }
        else if (yy >= zz) { a[1] = sqrt(max(yy, 0.0)); a[0] = (R[0][1] + R[1][0]) / (2 * (1 - c) * a[1]); a[2] = (R[1][2] + R[2][1]) / (2 * (1 - c) * a[1]) }
        else { a[2] = sqrt(max(zz, 0.0)); a[0] = (R[0][2] + R[2][0]) / (2 * (1 - c) * a[2]); a[1] = (R[1][2] + R[2][1]) / (2 * (1 - c) * a[2]) }
        val n = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
        val sign = if (a[0] * ax + a[1] * ay + a[2] * az < 0) -1.0 else 1.0           // the sense, as far as the antisymmetric part still tells
        return doubleArrayOf(sign * a[0] / n * th, sign * a[1] / n * th, sign * a[2] / n * th)
    }

    fun apply(R: Array<DoubleArray>, p: Vec3) = Vec3(
        R[0][0] * p.x + R[0][1] * p.y + R[0][2] * p.z,
        R[1][0] * p.x + R[1][1] * p.y + R[1][2] * p.z,
        R[2][0] * p.x + R[2][1] * p.y + R[2][2] * p.z)

    fun applyT(R: Array<DoubleArray>, p: Vec3) = Vec3(
        R[0][0] * p.x + R[1][0] * p.y + R[2][0] * p.z,
        R[0][1] * p.x + R[1][1] * p.y + R[2][1] * p.z,
        R[0][2] * p.x + R[1][2] * p.y + R[2][2] * p.z)
}

/** Pinhole + radial (k1, k2) model: f, cx, cy, k1, k2. Same model the Python solver uses. */
class Camera(@JvmField var f: Double, @JvmField var cx: Double, @JvmField var cy: Double, @JvmField var k1: Double, @JvmField var k2: Double) {
    fun copy() = Camera(f, cx, cy, k1, k2)
    fun toArray() = doubleArrayOf(f, cx, cy, k1, k2)

    /** Project a camera-frame point; returns null when behind the camera. */
    fun project(pc: Vec3): DoubleArray? {
        if (pc.z <= 1e-9) return null
        val xn = pc.x / pc.z; val yn = pc.y / pc.z
        val r2 = xn * xn + yn * yn
        val d = 1 + k1 * r2 + k2 * r2 * r2
        return doubleArrayOf(f * xn * d + cx, f * yn * d + cy)
    }

    /** Pixel -> undistorted normalised coordinates (iterative inverse of the radial model). */
    fun undistort(u: Double, v: Double): DoubleArray {
        val xd = (u - cx) / f; val yd = (v - cy) / f
        var x = xd; var y = yd
        for (i in 0 until 20) {
            val r2 = x * x + y * y
            val d = 1 + k1 * r2 + k2 * r2 * r2
            val nx = xd / d; val ny = yd / d
            if (abs(nx - x) < 1e-12 && abs(ny - y) < 1e-12) { x = nx; y = ny; break }
            x = nx; y = ny
        }
        return doubleArrayOf(x, y)
    }

    /** Undistorted normalised -> pixel. */
    fun distort(xn: Double, yn: Double): DoubleArray {
        val r2 = xn * xn + yn * yn
        val d = 1 + k1 * r2 + k2 * r2 * r2
        return doubleArrayOf(f * xn * d + cx, f * yn * d + cy)
    }
}

/** A camera pose: world -> camera, p_c = R p_w + t. */
class Pose6(@JvmField val r: DoubleArray, @JvmField val t: DoubleArray) {
    fun R() = Rot.fromVec(r)
    fun toCamera(p: Vec3): Vec3 { val q = Rot.apply(R(), p); return Vec3(q.x + t[0], q.y + t[1], q.z + t[2]) }
    fun centre(): Vec3 = -Rot.applyT(R(), Vec3(t[0], t[1], t[2]))
    fun copy() = Pose6(r.copyOf(), t.copyOf())
}

/**
 * Out-of-plane shape of the printed sheet. Paper that is not glued to something rigid bends by millimetres;
 * the solver estimates that bend as a quadratic surface (the model mrcal and Hagemann et al. use for
 * calibration boards). No constant or linear term: those are the pose's. Coefficients are in mm at the
 * normalised distance 1 = half the board diagonal from the board centre.
 */
class SheetWarp(@JvmField val cx: Double, @JvmField val cy: Double, @JvmField val half: Double, @JvmField val k: DoubleArray = DoubleArray(3)) {
    /** Height of the sheet at board position (x, y) in mm, positive towards the camera side (board z = -height). */
    fun height(x: Double, y: Double): Double {
        val u = (x - cx) / half; val v = (y - cy) / half
        return k[0] * u * u + k[1] * u * v + k[2] * v * v
    }
    fun isFlat() = k[0] == 0.0 && k[1] == 0.0 && k[2] == 0.0
    fun copy() = SheetWarp(cx, cy, half, k.copyOf())
    /** The board corner with this in-plane position, on the bent sheet (board frame, z into the board). */
    fun lift(p: Vec3) = Vec3(p.x, p.y, -height(p.x, p.y))

    /** Largest minus smallest height over the sheet (mm): how far from flat it is. */
    fun peakToValley(): Double {
        var lo = Double.MAX_VALUE; var hi = -Double.MAX_VALUE
        for (i in 0..20) for (j in 0..20) { val h = height(2 * cx * i / 20.0, 2 * cy * j / 20.0); if (h < lo) lo = h; if (h > hi) hi = h }
        return hi - lo
    }

    /** Local slope of the sheet at (x, y): tilt of the tangent plane in degrees. */
    fun slopeDeg(x: Double, y: Double): Double {
        val e = 1.0
        val sx = (height(x + e, y) - height(x - e, y)) / (2 * e); val sy = (height(x, y + e) - height(x, y - e)) / (2 * e)
        return Math.toDegrees(Math.atan(hypot(sx, sy)))
    }

    companion object {
        fun flat(widthMm: Double, heightMm: Double) = SheetWarp(widthMm / 2, heightMm / 2, hypot(widthMm, heightMm) / 2)
    }
}

/** Dense Cholesky solve of a symmetric positive-definite system A x = b (A is n x n row-major). */
object Linear {
    /** Lower-triangular Cholesky factor, or null when A is not positive definite. */
    fun cholesky(A: Array<DoubleArray>): Array<DoubleArray>? {
        val n = A.size
        val L = Array(n) { DoubleArray(n) }
        for (i in 0 until n) {
            val Li = L[i]
            for (j in 0..i) {
                var s = A[i][j]; val Lj = L[j]
                for (k in 0 until j) s -= Li[k] * Lj[k]
                if (i == j) { if (s <= 1e-300) return null; Li[i] = sqrt(s) } else Li[j] = s / Lj[j]
            }
        }
        return L
    }

    /** Solve L L^T x = b with a factor from [cholesky]. */
    fun cholSolve(L: Array<DoubleArray>, b: DoubleArray): DoubleArray {
        val n = b.size
        val y = DoubleArray(n)
        for (i in 0 until n) { var s = b[i]; val Li = L[i]; for (k in 0 until i) s -= Li[k] * y[k]; y[i] = s / Li[i] }
        val x = DoubleArray(n)
        for (i in n - 1 downTo 0) { var s = y[i]; for (k in i + 1 until n) s -= L[k][i] * x[k]; x[i] = s / L[i][i] }
        return x
    }

    fun choleskySolve(A: Array<DoubleArray>, b: DoubleArray): DoubleArray? = cholesky(A)?.let { cholSolve(it, b) }

    /** Inverse of a symmetric positive-definite matrix (for covariance): one factorisation, n solves. */
    fun inverseSpd(A: Array<DoubleArray>): Array<DoubleArray>? {
        val n = A.size
        val L = cholesky(A) ?: return null
        val inv = Array(n) { DoubleArray(n) }
        val e = DoubleArray(n)
        for (j in 0 until n) {
            e[j] = 1.0
            val col = cholSolve(L, e)
            e[j] = 0.0
            for (i in 0 until n) inv[i][j] = col[i]
        }
        return inv
    }

    /** Solve a general square system by Gaussian elimination with partial pivoting. */
    fun solve(A: Array<DoubleArray>, b: DoubleArray): DoubleArray? {
        val n = b.size
        val M = Array(n) { A[it].copyOf() }; val v = b.copyOf()
        for (c in 0 until n) {
            var p = c
            for (r in c + 1 until n) if (abs(M[r][c]) > abs(M[p][c])) p = r
            if (abs(M[p][c]) < 1e-300) return null
            if (p != c) { val tmp = M[p]; M[p] = M[c]; M[c] = tmp; val tv = v[p]; v[p] = v[c]; v[c] = tv }
            for (r in c + 1 until n) {
                val fct = M[r][c] / M[c][c]
                if (fct == 0.0) continue
                for (k in c until n) M[r][k] -= fct * M[c][k]
                v[r] -= fct * v[c]
            }
        }
        val x = DoubleArray(n)
        for (i in n - 1 downTo 0) { var s = v[i]; for (k in i + 1 until n) s -= M[i][k] * x[k]; x[i] = s / M[i][i] }
        return x
    }
}
