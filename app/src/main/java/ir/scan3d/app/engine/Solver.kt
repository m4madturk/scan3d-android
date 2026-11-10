package ir.scan3d.app.engine

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

/** Observations of one frame, in sensor-frame pixels. */
class FrameObs(
    val index: Int,                       // index into the input list
    val boardIds: IntArray, val boardPx: DoubleArray,   // ChArUco corner ids and x,y pairs
    val stickers: Map<Int, DoubleArray>,  // sticker id -> 8 values (4 corners)
    val isPhoto: Boolean,
)

class StickerResult(val id: Int, val views: Int, val centre: Vec3, val sigma: DoubleArray, val corners: List<Vec3>, val edgesMm: DoubleArray, val normalOut: Vec3)

class SolveResult(
    val camera: Camera, val poses: Array<Pose6?>, val stickers: List<StickerResult>,
    val sigma0: Double, val rmsBoard: Double, val rmsStickers: Double,
    val rejectedBoard: Int, val rejectedStickers: Int, val frameworkFrames: Int,
    val edgeBiasMm: Double, val warnings: List<String>, val corr: Array<DoubleArray>?,
    /** Estimated bend of the printed sheet (flat when it could not be estimated) and the 1-sigma of its coefficients (mm). */
    val warp: SheetWarp, val warpSigma: DoubleArray?,
    /** Board reprojection RMS of every input frame (px; NaN where the frame has no pose). */
    val frameRms: DoubleArray,
)

/**
 * Marker framework solver: Kotlin port of reconstruct.solve (v2).
 *   - board corners are fixed 3D points (mm), stickers are free 3D centres (diagonal intersection)
 *   - unknowns: camera (5), one pose per framework frame (6 each), one centre per sticker (3 each)
 *   - sparse Levenberg-Marquardt with Huber loss and Schur complement over the sticker block
 *   - 3-sigma rejection in up to three rounds, then covariance for precision and correlations
 *   - the sheet is NOT assumed flat: with 6 or more frames a quadratic bend (3 parameters, weak zero prior) is
 *     estimated along with everything else. On a real scan (paper on a carpet) the bend was 2.5 mm
 *     peak-to-valley and modelling it cut the board residual from 1.5 to 1.1 px.
 * Board frame: x, y in mm on the board, z into the board.
 */
class Solver(private val squareMm: Double, private val squaresX: Int = 7, private val targetMm: Double = 12.0, private val squaresY: Int = 10) {
    var log: (String) -> Unit = {}
    /** Estimate the sheet bend (needs at least [WARP_MIN_FRAMES] frames). */
    var estimateWarp = true

    fun boardPoint(id: Int): Vec3 = Vec3(((id % (squaresX - 1)) + 1) * squareMm, ((id / (squaresX - 1)) + 1) * squareMm, 0.0)
    fun boardPoint(id: Int, warp: SheetWarp): Vec3 = warp.lift(boardPoint(id))

    private class Obs(val frame: Int, val pointIdx: Int, val fixed: Vec3?, val u: Double, val v: Double) { var active = true }

    fun solve(frames: List<FrameObs>, w: Int, h: Int, prior: CameraPrior? = null, minViews: Int = 2): SolveResult {
        val warnings = ArrayList<String>()
        val photos = frames.filter { it.isPhoto && it.boardIds.size >= 6 }
        val framework = if (photos.isNotEmpty()) photos else frames.filter { it.boardIds.size >= 6 }
        if (photos.isEmpty()) warnings.add("no still photos; framework built from video frames")
        require(framework.isNotEmpty()) { "no frame shows the board" }

        // ---- initial camera and poses ------------------------------------------------------
        var cam = prior?.camera?.copy() ?: Camera(0.85 * maxOf(w, h), w / 2.0, h / 2.0, 0.0, 0.0)
        val poses = HashMap<Int, Pose6>()
        var lastGood: Pose6? = null
        val failed = ArrayList<Int>()
        for (fr in framework) {
            val obj = fr.boardIds.map { boardPoint(it) }; val img = (0 until fr.boardIds.size).map { doubleArrayOf(fr.boardPx[2 * it], fr.boardPx[2 * it + 1]) }
            var p = Pnp.solve(obj, img, cam)
            // the homography init can fail on noisy data: fall back to refining from the last good pose
            if ((p == null || Pnp.rms(obj, img, cam, p) > 25.0) && lastGood != null) { val q = Pnp.solve(obj, img, cam, lastGood); if (q != null && (p == null || Pnp.rms(obj, img, cam, q) < Pnp.rms(obj, img, cam, p))) p = q }
            if (p != null && Pnp.rms(obj, img, cam, p) < 25.0) { poses[fr.index] = p; lastGood = p } else failed.add(fr.index)
        }
        if (failed.isNotEmpty()) log("board pose failed for frames ${failed.map { it + 1 }}")
        val fw = framework.filter { poses.containsKey(it.index) }
        require(fw.size >= 2) { "fewer than 2 frames with a usable board pose" }
        if (prior == null) {
            // a few alternating rounds of camera refinement and pose refinement for a decent start
            for (round in 0 until 3) {
                cam = refineCameraOnly(fw, poses, cam)
                for (fr in fw) { val obj = fr.boardIds.map { boardPoint(it) }; val img = (0 until fr.boardIds.size).map { doubleArrayOf(fr.boardPx[2 * it], fr.boardPx[2 * it + 1]) }; poses[fr.index] = Pnp.refine(obj, img, cam, poses[fr.index]!!, 15) }
            }
        }

        // ---- sticker centres: diagonal intersection in undistorted coordinates, triangulated ----
        val centreObs = HashMap<Int, ArrayList<Pair<Int, DoubleArray>>>()
        for (fr in fw) for ((id, c) in fr.stickers) {
            if (quadArea(c) < 60) continue
            val ctr = diagonalCentre(c, cam) ?: continue
            centreObs.getOrPut(id) { ArrayList() }.add(fr.index to ctr)
        }
        val stickerIds = centreObs.filter { it.value.size >= minViews }.keys.sorted()
        val X = HashMap<Int, Vec3>()
        for (id in stickerIds) triangulate(centreObs[id]!!, poses, cam)?.let { X[id] = it }
        val ids = stickerIds.filter { X.containsKey(it) }
        log("framework ${fw.size} frames, ${ids.size} stickers (seen in >= $minViews frames; all detected ids: ${centreObs.keys.sorted()})")

        // ---- bundle adjustment ------------------------------------------------------------
        val fIdx = fw.mapIndexed { i, f -> f.index to i }.toMap()
        val sIdx = ids.mapIndexed { i, id -> id to i }.toMap()
        val obs = ArrayList<Obs>()
        for (fr in fw) {
            for (i in fr.boardIds.indices) obs.add(Obs(fIdx[fr.index]!!, -1, boardPoint(fr.boardIds[i]), fr.boardPx[2 * i], fr.boardPx[2 * i + 1]))
            for ((id, c) in fr.stickers) { val si = sIdx[id] ?: continue; val ctr = diagonalCentre(c, cam) ?: continue; obs.add(Obs(fIdx[fr.index]!!, si, null, ctr[0], ctr[1])) }
        }
        val nW = if (estimateWarp && fw.size >= WARP_MIN_FRAMES) 3 else 0
        val state = State(cam.copy(), fw.map { poses[it.index]!!.copy() }.toTypedArray(), ids.map { X[it]!! }.toTypedArray(),
            SheetWarp.flat(squaresX * squareMm, squaresY * squareMm), nW)
        var sigma0 = 1.0
        var rejB = 0; var rejS = 0
        for (round in 0 until 3) {
            sigma0 = lm(state, obs, prior, 40)
            val lim = maxOf(1.0, 3.0 * sigma0)
            var nb = 0; var ns = 0
            for (o in obs) if (o.active) { val e = resid(state, o) ?: continue; if (sqrt(e[0] * e[0] + e[1] * e[1]) > lim) { o.active = false; if (o.fixed != null) nb++ else ns++ } }
            rejB += nb; rejS += ns
            log("BA round ${round + 1}: sigma0 %.3f px, rejected $nb+$ns".format(sigma0))
            if (nb + ns == 0) break
        }
        // final statistics
        var sb = 0.0; var cb = 0; var ss = 0.0; var cs = 0
        for (o in obs) if (o.active) { val e = resid(state, o) ?: continue; val d = e[0] * e[0] + e[1] * e[1]; if (o.fixed != null) { sb += d; cb++ } else { ss += d; cs++ } }
        val rmsB = if (cb > 0) sqrt(sb / cb) else Double.NaN; val rmsS = if (cs > 0) sqrt(ss / cs) else Double.NaN

        // ---- covariance (full dense, fine for our sizes) ----------------------------------
        val (JtJ, _) = normalEquations(state, obs, prior)
        val cov = Linear.inverseSpd(JtJ)
        val sig = Array(state.pts.size) { DoubleArray(3) { Double.NaN } }
        var corr: Array<DoubleArray>? = null
        var warpSigma: DoubleArray? = null
        if (cov != null) {
            if (nW > 0) { val b = 5 + 6 * state.poses.size + 3 * state.pts.size; warpSigma = DoubleArray(3) { sqrt(maxOf(0.0, cov[b + it][b + it])) * sigma0 } }
            for (i in state.pts.indices) { val b = 5 + 6 * state.poses.size + 3 * i; for (k in 0 until 3) sig[i][k] = sqrt(maxOf(0.0, cov[b + k][b + k]) ) * sigma0 }
            corr = Array(5) { a -> DoubleArray(5) { b -> cov[a][b] / sqrt(maxOf(1e-30, cov[a][a] * cov[b][b])) } }
            for (a in 0 until 5) for (b in a + 1 until 5) if (abs(corr[a][b]) > 0.95) warnings.add("intrinsics ${NAMES[a]}/${NAMES[b]} correlation %.2f: add rotated and closer tilted photos".format(corr[a][b]))
        }
        if (photos.isNotEmpty() && fw.size < 8) warnings.add("only ${fw.size} framework photos; 12-30 recommended")
        if (nW > 0) log("sheet bend: %.2f mm peak-to-valley (coefficients %.2f %.2f %.2f mm, sigma %s)".format(state.warp.peakToValley(), state.warp.k[0], state.warp.k[1], state.warp.k[2],
            warpSigma?.joinToString(" ") { "%.2f".format(it) } ?: "-"))
        else log("sheet bend not estimated (${fw.size} frames < $WARP_MIN_FRAMES): assuming a flat sheet")

        // ---- corners for normals and blur diagnostic -----------------------------------------
        val camC = state.poses.map { it.centre() }
        val results = ArrayList<StickerResult>()
        var edgeSum = 0.0; var edgeN = 0
        for ((i, id) in ids.withIndex()) {
            val seen = fw.filter { it.stickers.containsKey(id) && centreObs[id]!!.any { o -> o.first == it.index } }
            val corners = ArrayList<Vec3>()
            for (k in 0 until 4) {
                val o = seen.mapNotNull { fr -> fIdx[fr.index]?.let { fi -> fi to doubleArrayOf(fr.stickers[id]!![2 * k], fr.stickers[id]!![2 * k + 1]) } }
                corners.add(triangulateIdx(o, state) ?: state.pts[i])
            }
            val edges = DoubleArray(4) { k -> (corners[(k + 1) % 4] - corners[k]).norm() }
            edges.forEach { edgeSum += it; edgeN++ }
            var n = (corners[1] - corners[0]).cross(corners[3] - corners[0]).unit()
            val c = state.pts[i]
            val toCam = seen.mapNotNull { fr -> fIdx[fr.index] }.map { (camC[it] - c).dot(n) }.average()
            if (toCam < 0) n = -n
            results.add(StickerResult(id, seen.size, c, sig[i], corners, edges, n))
        }
        val edgeBias = if (edgeN > 0) edgeSum / edgeN - targetMm else Double.NaN
        for (r in results) log("sticker ${r.id}: ${r.views} views, height above board %.1f mm, sigma %.2f mm".format(-r.centre.z, r.sigma.maxOrNull() ?: Double.NaN))

        // ---- other frames (video): localise against the frozen framework ------------------
        val allPoses = arrayOfNulls<Pose6>(frames.size)
        for (fr in fw) allPoses[fr.index] = state.poses[fIdx[fr.index]!!]
        val centres = ids.mapIndexed { i, id -> id to state.pts[i] }.toMap()
        for (fr in frames) if (allPoses[fr.index] == null && fr.boardIds.size >= 6) {
            val obj = ArrayList<Vec3>(); val img = ArrayList<DoubleArray>()
            for (i in fr.boardIds.indices) { obj.add(boardPoint(fr.boardIds[i], state.warp)); img.add(doubleArrayOf(fr.boardPx[2 * i], fr.boardPx[2 * i + 1])) }
            for ((id, c) in fr.stickers) { val p = centres[id] ?: continue; val ctr = diagonalCentre(c, state.cam) ?: continue; obj.add(p); img.add(ctr) }
            // start from the planar solution, then refine on the bent sheet + stickers
            val flat = fr.boardIds.map { boardPoint(it) }; val flatImg = img.subList(0, flat.size)
            allPoses[fr.index] = Pnp.solve(flat, flatImg, state.cam)?.let { Pnp.refine(obj, img, state.cam, it, 20) }
        }
        val frameRms = DoubleArray(frames.size) { Double.NaN }
        for (fr in frames) { val p = allPoses[fr.index] ?: continue
            var ss = 0.0; var n = 0
            for (i in fr.boardIds.indices) { val pr = state.cam.project(p.toCamera(boardPoint(fr.boardIds[i], state.warp))) ?: continue
                val du = pr[0] - fr.boardPx[2 * i]; val dv = pr[1] - fr.boardPx[2 * i + 1]; ss += du * du + dv * dv; n++ }
            if (n > 0) frameRms[fr.index] = sqrt(ss / n) }
        return SolveResult(state.cam, allPoses, results, sigma0, rmsB, rmsS, rejB, rejS, fw.size, edgeBias, warnings, corr, state.warp, warpSigma, frameRms)
    }

    // ------------------------------------------------------------------ internals
    /** nW = number of sheet-bend parameters in the state vector (0 = flat sheet, 3 = quadratic bend). */
    private class State(var cam: Camera, val poses: Array<Pose6>, val pts: Array<Vec3>, val warp: SheetWarp, val nW: Int) {
        val size get() = 5 + 6 * poses.size + 3 * pts.size + nW
        val warpBase get() = 5 + 6 * poses.size + 3 * pts.size
    }

    private fun point(s: State, o: Obs): Vec3 = if (o.fixed != null) (if (s.nW > 0) s.warp.lift(o.fixed) else o.fixed) else s.pts[o.pointIdx]

    private fun resid(s: State, o: Obs): DoubleArray? {
        val pr = s.cam.project(s.poses[o.frame].toCamera(point(s, o))) ?: return null
        return doubleArrayOf(pr[0] - o.u, pr[1] - o.v)
    }

    private fun huberW(r: Double, delta: Double) = if (r <= delta) 1.0 else delta / r

    /** Full normal equations (dense). Returns (JtJ, Jtr) with Huber weights and prior rows. */
    private fun normalEquations(s: State, obs: List<Obs>, prior: CameraPrior?): Pair<Array<DoubleArray>, DoubleArray> {
        val nF = s.poses.size; val nX = s.pts.size; val n = s.size
        val A = Array(n) { DoubleArray(n) }; val g = DoubleArray(n)
        val d = 1e-6
        for (o in obs) {
            if (!o.active) continue
            val r0 = resid(s, o) ?: continue
            val rn = sqrt(r0[0] * r0[0] + r0[1] * r0[1]); val wgt = huberW(rn, 1.0)
            val cols = ArrayList<Int>(); val J = ArrayList<DoubleArray>()   // column index, (dr/dp for u, v)
            val pw = point(s, o)
            val pc = s.poses[o.frame].toCamera(pw)
            // camera
            for (k in 0 until 5) { val c = s.cam.copy(); when (k) { 0 -> c.f += d; 1 -> c.cx += d; 2 -> c.cy += d; 3 -> c.k1 += d; else -> c.k2 += d }
                val pr = c.project(pc) ?: continue
                cols.add(k); J.add(doubleArrayOf((pr[0] - o.u - r0[0]) / d, (pr[1] - o.v - r0[1]) / d)) }
            // pose
            for (k in 0 until 6) { val q = s.poses[o.frame].copy(); if (k < 3) q.r[k] += d else q.t[k - 3] += d
                val pr = s.cam.project(q.toCamera(pw)) ?: continue
                cols.add(5 + 6 * o.frame + k); J.add(doubleArrayOf((pr[0] - o.u - r0[0]) / d, (pr[1] - o.v - r0[1]) / d)) }
            // sheet bend (board corners only): the corner moves along the board normal
            if (o.fixed != null && s.nW > 0) for (k in 0 until s.nW) {
                val keep = s.warp.k[k]; s.warp.k[k] = keep + 1e-4
                val pr = s.cam.project(s.poses[o.frame].toCamera(s.warp.lift(o.fixed))); s.warp.k[k] = keep
                if (pr == null) continue
                cols.add(s.warpBase + k); J.add(doubleArrayOf((pr[0] - o.u - r0[0]) / 1e-4, (pr[1] - o.v - r0[1]) / 1e-4)) }
            // point
            if (o.fixed == null) for (k in 0 until 3) { val p = s.pts[o.pointIdx]; val q = Vec3(p.x, p.y, p.z); when (k) { 0 -> q.x += d; 1 -> q.y += d; else -> q.z += d }
                val pr = s.cam.project(s.poses[o.frame].toCamera(q)) ?: continue
                cols.add(5 + 6 * nF + 3 * o.pointIdx + k); J.add(doubleArrayOf((pr[0] - o.u - r0[0]) / d, (pr[1] - o.v - r0[1]) / d)) }
            for (a in cols.indices) {
                val ca = cols[a]; val ja = J[a]
                g[ca] += wgt * (ja[0] * r0[0] + ja[1] * r0[1])
                for (b in cols.indices) { val cb = cols[b]; val jb = J[b]; A[ca][cb] += wgt * (ja[0] * jb[0] + ja[1] * jb[1]) }
            }
        }
        prior?.let { pr ->
            val v = pr.camera.toArray(); val cur = s.cam.toArray()
            for (k in 0 until 5) { val sg = pr.sigma[k]; if (sg > 0) { A[k][k] += 1 / (sg * sg); g[k] += (cur[k] - v[k]) / (sg * sg) } }
        }
        // weak zero prior on the bend: keeps it at "flat" when the views cannot tell
        for (k in 0 until s.nW) { val b = s.warpBase + k; A[b][b] += 1 / (WARP_PRIOR_MM * WARP_PRIOR_MM); g[b] += s.warp.k[k] / (WARP_PRIOR_MM * WARP_PRIOR_MM) }
        return A to g
    }

    private fun cost(s: State, obs: List<Obs>, prior: CameraPrior?): Double {
        var c = 0.0
        for (o in obs) if (o.active) { val r = resid(s, o) ?: return Double.MAX_VALUE; val rn = sqrt(r[0] * r[0] + r[1] * r[1]); c += if (rn <= 1.0) rn * rn else 2 * rn - 1 }
        prior?.let { pr -> val v = pr.camera.toArray(); val cur = s.cam.toArray(); for (k in 0 until 5) if (pr.sigma[k] > 0) { val z = (cur[k] - v[k]) / pr.sigma[k]; c += z * z } }
        for (k in 0 until s.nW) { val z = s.warp.k[k] / WARP_PRIOR_MM; c += z * z }
        return c
    }

    /** Levenberg-Marquardt; returns sigma0 (a-posteriori std of unit weight, px). */
    private fun lm(s: State, obs: List<Obs>, prior: CameraPrior?, iters: Int): Double {
        var lambda = 1e-3
        var c0 = cost(s, obs, prior)
        for (it in 0 until iters) {
            val (A, g) = normalEquations(s, obs, prior)
            var improved = false
            for (tries in 0 until 8) {
                val n = g.size
                val Aug = Array(n) { i -> DoubleArray(n) { j -> A[i][j] + if (i == j) lambda * (1 + A[i][i]) else 0.0 } }
                val dx = Linear.choleskySolve(Aug, DoubleArray(n) { -g[it] }) ?: break
                val trial = State(s.cam.copy(), Array(s.poses.size) { s.poses[it].copy() }, Array(s.pts.size) { Vec3(s.pts[it].x, s.pts[it].y, s.pts[it].z) }, s.warp.copy(), s.nW)
                for (k in 0 until s.nW) trial.warp.k[k] += dx[s.warpBase + k]
                trial.cam.f += dx[0]; trial.cam.cx += dx[1]; trial.cam.cy += dx[2]; trial.cam.k1 += dx[3]; trial.cam.k2 += dx[4]
                for (f in s.poses.indices) for (k in 0 until 3) { trial.poses[f].r[k] += dx[5 + 6 * f + k]; trial.poses[f].t[k] += dx[5 + 6 * f + 3 + k] }
                for (p in s.pts.indices) { val b = 5 + 6 * s.poses.size + 3 * p; trial.pts[p].x += dx[b]; trial.pts[p].y += dx[b + 1]; trial.pts[p].z += dx[b + 2] }
                val c1 = cost(trial, obs, prior)
                if (c1 < c0) {
                    s.cam = trial.cam; for (f in s.poses.indices) { s.poses[f].r.indices.forEach { k -> s.poses[f].r[k] = trial.poses[f].r[k]; s.poses[f].t[k] = trial.poses[f].t[k] } }
                    for (p in s.pts.indices) { s.pts[p].x = trial.pts[p].x; s.pts[p].y = trial.pts[p].y; s.pts[p].z = trial.pts[p].z }
                    for (k in 0 until s.nW) s.warp.k[k] = trial.warp.k[k]
                    val rel = (c0 - c1) / maxOf(c0, 1e-12); c0 = c1; lambda = maxOf(lambda / 3, 1e-9); improved = true
                    if (rel < 1e-8) return sigma0(s, obs)
                    break
                } else lambda *= 5
            }
            if (!improved) break
        }
        return sigma0(s, obs)
    }

    private fun sigma0(s: State, obs: List<Obs>): Double {
        var ss = 0.0; var n = 0
        for (o in obs) if (o.active) { val r = resid(s, o) ?: continue; ss += r[0] * r[0] + r[1] * r[1]; n += 2 }
        val dof = maxOf(1, n - s.size)
        return sqrt(ss / dof)
    }

    private fun refineCameraOnly(fw: List<FrameObs>, poses: Map<Int, Pose6>, cam: Camera): Camera {
        // Gauss-Newton on (f, cx, cy, k1, k2) with poses fixed; cheap and stabilises the start
        var c = cam.copy()
        for (it in 0 until 10) {
            val A = Array(5) { DoubleArray(5) }; val g = DoubleArray(5)
            val d = 1e-6
            for (fr in fw) { val p = poses[fr.index] ?: continue
                for (i in fr.boardIds.indices) {
                    val P = boardPoint(fr.boardIds[i]); val pc = p.toCamera(P)
                    val r0 = c.project(pc) ?: continue; r0[0] -= fr.boardPx[2 * i]; r0[1] -= fr.boardPx[2 * i + 1]
                    val J = Array(5) { DoubleArray(2) }
                    for (k in 0 until 5) { val cc = c.copy(); when (k) { 0 -> cc.f += d; 1 -> cc.cx += d; 2 -> cc.cy += d; 3 -> cc.k1 += d; else -> cc.k2 += d }
                        val pr = cc.project(pc) ?: continue; J[k][0] = (pr[0] - fr.boardPx[2 * i] - r0[0]) / d; J[k][1] = (pr[1] - fr.boardPx[2 * i + 1] - r0[1]) / d }
                    for (a in 0 until 5) { g[a] += J[a][0] * r0[0] + J[a][1] * r0[1]; for (b in 0 until 5) A[a][b] += J[a][0] * J[b][0] + J[a][1] * J[b][1] }
                } }
            for (k in 0 until 5) A[k][k] += 1e-6 * (1 + A[k][k])
            val dx = Linear.choleskySolve(A, DoubleArray(5) { -g[it] }) ?: break
            c.f += dx[0]; c.cx += dx[1]; c.cy += dx[2]; c.k1 += dx[3]; c.k2 += dx[4]
            if (dx.all { abs(it) < 1e-7 }) break
        }
        return c
    }

    fun diagonalCentre(c: DoubleArray, cam: Camera): DoubleArray? {
        val u = Array(4) { cam.undistort(c[2 * it], c[2 * it + 1]) }
        // lines through (0,2) and (1,3) in normalised coordinates: homogeneous cross products
        val l1 = cross3(doubleArrayOf(u[0][0], u[0][1], 1.0), doubleArrayOf(u[2][0], u[2][1], 1.0))
        val l2 = cross3(doubleArrayOf(u[1][0], u[1][1], 1.0), doubleArrayOf(u[3][0], u[3][1], 1.0))
        val p = cross3(l1, l2)
        if (abs(p[2]) < 1e-12) return null
        return cam.distort(p[0] / p[2], p[1] / p[2])
    }

    private fun cross3(a: DoubleArray, b: DoubleArray) = doubleArrayOf(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    private fun quadArea(c: DoubleArray): Double {
        var a = 0.0
        for (k in 0 until 4) { val j = (k + 1) % 4; a += c[2 * k] * c[2 * j + 1] - c[2 * j] * c[2 * k + 1] }
        return abs(a) / 2
    }

    /** Linear triangulation (DLT) of a point from pixel observations in several frames. */
    private fun triangulate(o: List<Pair<Int, DoubleArray>>, poses: Map<Int, Pose6>, cam: Camera): Vec3? {
        val rows = ArrayList<DoubleArray>()
        for ((fi, px) in o) {
            val p = poses[fi] ?: continue; val R = p.R(); val (xn, yn) = cam.undistort(px[0], px[1]).let { it[0] to it[1] }
            // P = [R | t] in normalised coords
            val P = arrayOf(doubleArrayOf(R[0][0], R[0][1], R[0][2], p.t[0]), doubleArrayOf(R[1][0], R[1][1], R[1][2], p.t[1]), doubleArrayOf(R[2][0], R[2][1], R[2][2], p.t[2]))
            rows.add(DoubleArray(4) { xn * P[2][it] - P[0][it] }); rows.add(DoubleArray(4) { yn * P[2][it] - P[1][it] })
        }
        if (rows.size < 4) return null
        return nullVector4(rows)
    }

    private fun triangulateIdx(o: List<Pair<Int, DoubleArray>>, s: State): Vec3? {
        val m = HashMap<Int, Pose6>(); for ((fi, _) in o) m[fi] = s.poses[fi]
        return triangulate(o, m, s.cam)
    }

    private fun nullVector4(rows: List<DoubleArray>): Vec3? {
        val M = Array(4) { DoubleArray(4) }
        for (r in rows) for (i in 0 until 4) for (j in 0 until 4) M[i][j] += r[i] * r[j]
        var tr = 0.0; for (i in 0 until 4) tr += M[i][i]
        for (i in 0 until 4) M[i][i] += tr * 1e-12 + 1e-15
        var v = doubleArrayOf(0.5, 0.5, 0.5, 0.5)
        for (it in 0 until 50) { val w = Linear.choleskySolve(M, v) ?: return null; var nn = 0.0; for (x in w) nn += x * x; nn = sqrt(nn); if (nn == 0.0) return null; v = DoubleArray(4) { w[it] / nn } }
        if (abs(v[3]) < 1e-12) return null
        return Vec3(v[0] / v[3], v[1] / v[3], v[2] / v[3])
    }

    companion object {
        val NAMES = arrayOf("f", "cx", "cy", "k1", "k2")
        const val WARP_MIN_FRAMES = 6
        const val WARP_PRIOR_MM = 15.0
    }
}

/** Camera profile used as a weighted prior (from a calibration session). */
class CameraPrior(val camera: Camera, val sigma: DoubleArray)
