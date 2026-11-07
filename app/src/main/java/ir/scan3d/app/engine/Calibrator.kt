package ir.scan3d.app.engine

import kotlin.math.abs
import kotlin.math.sqrt

class CalibResult(val camera: Camera, val sigma: DoubleArray, val rms: Double, val frames: Int, val rolled: Int,
                  val coverage: Int, val coverageCells: Int, val model: String, val warnings: List<String>, val perView: DoubleArray)

/**
 * Camera profile from a calibration session (port of calib_session.py): board-only bundle
 * adjustment with model selection (k1 / k1 k2), per-parameter 1-sigma from the covariance,
 * coverage and roll checks. Pure Kotlin.
 */
class Calibrator(private val squareMm: Double, private val squaresX: Int = 7) {
    var log: (String) -> Unit = {}

    fun calibrate(frames: List<FrameObs>, w: Int, h: Int, rolledFlags: BooleanArray): CalibResult {
        val usable = frames.filter { it.boardIds.size >= 12 }
        require(usable.size >= 10) { "only ${usable.size} usable frames; need at least 10 (25 recommended)" }
        val solver = Solver(squareMm, squaresX)
        var best: Pair<String, SolveResult>? = null
        for ((name, fixK2) in listOf("k1" to true, "k1 k2" to false)) {
            val r = solver.solve(usable.map { FrameObs(it.index, it.boardIds, it.boardPx, emptyMap(), true) }, w, h,
                prior = if (fixK2) CameraPrior(Camera(0.85 * maxOf(w, h), w / 2.0, h / 2.0, 0.0, 0.0), doubleArrayOf(0.0, 0.0, 0.0, 0.0, 1e-9)) else null)
            log("model $name: sigma0 %.4f px".format(r.sigma0))
            if (best == null || r.sigma0 < best.second.sigma0 * 0.97) best = name to r
        }
        val (model, r) = best!!
        // parameter sigmas from the correlation-normalised covariance are not exposed; recompute via a short BA on the final state
        val sig = DoubleArray(5)
        r.corr?.let { /* placeholder: filled below */ }
        val sigmas = paramSigmas(usable, w, h, r)
        for (k in 0 until 5) sig[k] = sigmas[k]
        // coverage
        val gx = 4; val gy = 6; val cells = IntArray(gx * gy)
        for (fr in usable) for (i in fr.boardIds.indices) { val x = fr.boardPx[2 * i]; val y = fr.boardPx[2 * i + 1]
            cells[minOf((y / h * gy).toInt(), gy - 1) * gx + minOf((x / w * gx).toInt(), gx - 1)]++ }
        val covered = cells.count { it >= 3 }
        val rolled = usable.count { rolledFlags.getOrElse(it.index) { false } }
        val tilts = r.poses.filterNotNull().map { p -> val R = p.R(); Math.toDegrees(Math.acos(abs(R[2][2]).coerceIn(0.0, 1.0))) }.sorted()
        val warnings = ArrayList<String>()
        if (rolled < 4) warnings.add("only $rolled rolled (landscape) frames; principal point weakly determined")
        if (covered < gx * gy - 2) warnings.add("frame coverage $covered/${gx * gy} cells; distortion poorly constrained at the edges")
        if (tilts.isNotEmpty() && tilts[(tilts.size * 3) / 4] < 25) warnings.add("few tilted views; focal length and distortion are correlated")
        if (r.sigma0 > 0.6) warnings.add("RMS %.2f px is high: blur, bent board, or focus changed between frames".format(r.sigma0))
        val perView = DoubleArray(usable.size)
        for ((i, fr) in usable.withIndex()) { val p = r.poses[fr.index] ?: continue
            val obj = fr.boardIds.map { solver.boardPoint(it) }; val img = (0 until fr.boardIds.size).map { doubleArrayOf(fr.boardPx[2 * it], fr.boardPx[2 * it + 1]) }
            perView[i] = Pnp.rms(obj, img, r.camera, p) }
        return CalibResult(r.camera, sig, r.sigma0, usable.size, rolled, covered, gx * gy, model, warnings, perView)
    }

    /** 1-sigma of the 5 intrinsics: diagonal of sigma0^2 (J^T J)^-1 restricted by the Schur complement. */
    private fun paramSigmas(frames: List<FrameObs>, w: Int, h: Int, r: SolveResult): DoubleArray {
        // numeric Jacobian of all board residuals w.r.t. [cam(5), poses(6 each)]
        val fw = frames.filter { r.poses[it.index] != null }
        val nF = fw.size; val n = 5 + 6 * nF
        val A = Array(n) { DoubleArray(n) }
        val d = 1e-6
        for ((fi, fr) in fw.withIndex()) {
            val p = r.poses[fr.index]!!
            for (i in fr.boardIds.indices) {
                val P = Vec3(((fr.boardIds[i] % (squaresX - 1)) + 1) * squareMm, ((fr.boardIds[i] / (squaresX - 1)) + 1) * squareMm, 0.0)
                val r0 = r.camera.project(p.toCamera(P)) ?: continue
                val cols = IntArray(11) { if (it < 5) it else 5 + 6 * fi + (it - 5) }
                val J = Array(11) { DoubleArray(2) }
                for (k in 0 until 5) { val c = r.camera.copy(); when (k) { 0 -> c.f += d; 1 -> c.cx += d; 2 -> c.cy += d; 3 -> c.k1 += d; else -> c.k2 += d }
                    val pr = c.project(p.toCamera(P)) ?: continue; J[k][0] = (pr[0] - r0[0]) / d; J[k][1] = (pr[1] - r0[1]) / d }
                for (k in 0 until 6) { val q = p.copy(); if (k < 3) q.r[k] += d else q.t[k - 3] += d
                    val pr = r.camera.project(q.toCamera(P)) ?: continue; J[5 + k][0] = (pr[0] - r0[0]) / d; J[5 + k][1] = (pr[1] - r0[1]) / d }
                for (a in 0 until 11) for (b in 0 until 11) A[cols[a]][cols[b]] += J[a][0] * J[b][0] + J[a][1] * J[b][1]
            }
        }
        for (i in 0 until n) A[i][i] += 1e-9 * (1 + A[i][i])
        val inv = Linear.inverseSpd(A) ?: return DoubleArray(5) { Double.NaN }
        return DoubleArray(5) { sqrt(maxOf(0.0, inv[it][it])) * r.sigma0 }
    }
}
