package ir.scan3d.app

import org.opencv.calib3d.Calib3d
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.opencv.core.MatOfInt
import org.opencv.core.MatOfPoint2f
import org.opencv.core.MatOfPoint3f
import org.opencv.core.Point
import org.opencv.core.Point3
import org.opencv.core.Size
import org.opencv.core.TermCriteria
import org.opencv.imgproc.Imgproc
import org.opencv.objdetect.ArucoDetector
import org.opencv.objdetect.CharucoBoard
import org.opencv.objdetect.CharucoDetector
import org.opencv.objdetect.DetectorParameters
import org.opencv.objdetect.Objdetect
import org.opencv.objdetect.RefineParameters
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sqrt

/** One sticker: id and its 4 corners in full-resolution pixel coordinates (sensor frame). */
data class Sticker(val id: Int, val corners: FloatArray)

/** ChArUco corners seen in a frame. */
data class BoardObs(val ids: IntArray, val pts: FloatArray)

/**
 * Where the phone is, relative to the board: azimuth (clock direction, 0 = 12 o'clock) and elevation
 * in degrees, distance in metres, and the roll of the camera about its optical axis relative to an
 * upright hold (90 = phone held sideways), measured against the board, not gravity.
 */
data class Pose(val az: Double, val el: Double, val dist: Double, val rvec: DoubleArray, val tvec: DoubleArray, val roll: Double = 0.0)

/**
 * Live preview result: sticker ids, number of board corners and (if the board is seen) the pose.
 * quads / outline are in preview-image pixels (w x h), for drawing what was detected over the preview.
 */
data class Live(val ids: List<Int>, val nBoard: Int, val pose: Pose?, val stickerSidePx: Double? = null,
                val quads: List<FloatArray> = emptyList(), val outline: FloatArray? = null, val w: Int = 0, val h: Int = 0)

data class Analysis(
    val width: Int, val height: Int,
    val stickers: List<Sticker>, val board: BoardObs?,
    val sharpness: Double, val pose: Pose?, val millis: Long,
)

/**
 * Same measurements as the web app, on the phone's full-resolution frames.
 * Detection runs on a copy no larger than DETECT_MAX px; corners are then refined on the
 * full image with the same gradient-orthogonality iteration the web app uses.
 */
object Detector {
    const val SQUARES_X = 7
    const val SQUARES_Y = 10
    const val SQUARE_M = 0.025
    const val MARKER_M = 0.018
    const val TARGET_MM = 12.0
    const val DETECT_MAX = 2000.0

    private val refine = RefineParameters(10f, 3f, true)
    private val targetDet by lazy {
        ArucoDetector(Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_250), DetectorParameters(), refine)
    }
    private val board by lazy {
        CharucoBoard(Size(SQUARES_X.toDouble(), SQUARES_Y.toDouble()), SQUARE_M.toFloat(), MARKER_M.toFloat(),
            Objdetect.getPredefinedDictionary(Objdetect.DICT_5X5_100))
    }
    private val boardDet by lazy { CharucoDetector(board) }

    /** Fast pass for the live preview (gray, already small). Returns sticker ids and board corner count. */
    fun quick(gray: Mat): Pair<List<Int>, Int> { val l = live(gray, null); return l.ids to l.nBoard }

    /**
     * Preview-resolution detection plus a quick board pose (approximate focal length, no distortion).
     * The board is detected on a half-size image (fast, its 18 mm markers stay large enough); the 12 mm
     * stickers on the full preview. stickerSidePx = median sticker side, for a rough distance when the
     * board is out of view.
     */
    fun live(gray: Mat, focalPx: Double?): Live {
        val corners = ArrayList<Mat>(); val ids = Mat()
        targetDet.detectMarkers(gray, corners, ids)
        val idList = ArrayList<Int>()
        if (!ids.empty()) for (i in 0 until ids.rows()) idList.add(ids.get(i, 0)[0].toInt())
        val sides = ArrayList<Double>(); val quads = ArrayList<FloatArray>()
        for (c in corners) {
            val v = FloatArray(8); c.get(0, 0, v); quads.add(v)
            var sum = 0.0; for (k in 0 until 4) { val a = k * 2; val b = ((k + 1) % 4) * 2; sum += hypot((v[a] - v[b]).toDouble(), (v[a + 1] - v[b + 1]).toDouble()) }
            sides.add(sum / 4)
        }
        corners.forEach { it.release() }; ids.release()
        val stickerSide = if (sides.isEmpty()) null else sides.sorted()[sides.size / 2]
        val small = Mat(); Imgproc.resize(gray, small, Size(), 0.5, 0.5, Imgproc.INTER_AREA)
        val cc = Mat(); val cid = Mat()
        boardDet.detectBoard(small, cc, cid)
        val nBoard = if (cid.empty()) 0 else cid.rows()
        var pose: Pose? = null
        if (nBoard >= 6) {
            val bids = IntArray(nBoard) { cid.get(it, 0)[0].toInt() }
            val pts = FloatArray(2 * nBoard); for (i in 0 until nBoard) { val v = cc.get(i, 0); pts[2 * i] = (2 * v[0] + 0.5).toFloat(); pts[2 * i + 1] = (2 * v[1] + 0.5).toFloat() }
            pose = try { boardPose(BoardObs(bids, pts), gray.cols(), gray.rows(), focalPx) } catch (_: Exception) { null }
        }
        cc.release(); cid.release(); small.release()
        // outline of the printed sheet as the live pose sees it (pinhole, no distortion: it is only drawn)
        val outline = pose?.let { p ->
            val f = focalPx ?: (0.85 * max(gray.cols(), gray.rows()))
            val R = ir.scan3d.app.engine.Rot.fromVec(p.rvec); val o = FloatArray(8); var ok = true
            val bw = SQUARES_X * SQUARE_M; val bh = SQUARES_Y * SQUARE_M
            for ((k, c) in listOf(0.0 to 0.0, bw to 0.0, bw to bh, 0.0 to bh).withIndex()) {
                val x = R[0][0] * c.first + R[0][1] * c.second + p.tvec[0]; val y = R[1][0] * c.first + R[1][1] * c.second + p.tvec[1]; val z = R[2][0] * c.first + R[2][1] * c.second + p.tvec[2]
                if (z <= 1e-6) { ok = false; break }
                o[2 * k] = (f * x / z + gray.cols() / 2.0).toFloat(); o[2 * k + 1] = (f * y / z + gray.rows() / 2.0).toFloat()
            }
            if (ok) o else null
        }
        return Live(idList, nBoard, pose, stickerSide, quads, outline, gray.cols(), gray.rows())
    }

    /** Full analysis of a captured frame (gray, full resolution). */
    fun analyze(gray: Mat, focalGuessPx: Double?): Analysis {
        val t0 = System.currentTimeMillis()
        val w = gray.cols(); val h = gray.rows()
        val sc = min(1.0, DETECT_MAX / max(w, h))
        val small = Mat()
        if (sc < 1.0) Imgproc.resize(gray, small, Size(Math.round(w * sc).toDouble(), Math.round(h * sc).toDouble()), 0.0, 0.0, Imgproc.INTER_AREA)
        else gray.copyTo(small)

        val lap = Mat(); Imgproc.Laplacian(small, lap, CvType.CV_64F)
        val mean = MatOfDouble(); val sd = MatOfDouble(); Core.meanStdDev(lap, mean, sd)
        val sharp = sd.toArray()[0].let { it * it }
        lap.release(); mean.release(); sd.release()

        val corners = ArrayList<Mat>(); val ids = Mat()
        targetDet.detectMarkers(small, corners, ids)
        val stickers = ArrayList<Sticker>()
        if (!ids.empty()) {
            for (i in 0 until min(ids.rows(), corners.size)) {
                val c = corners[i]
                val pts = FloatArray(8)
                for (k in 0 until 4) { val p = c.get(0, k); pts[2 * k] = up(p[0], sc); pts[2 * k + 1] = up(p[1], sc) }
                // window ~ a tenth of the sticker side: big enough to average out sharpening / noise-reduction
                // texture of phone JPEGs, small enough to stay clear of the marker's inner bits
                val side = (0 until 4).sumOf { k -> hypot((pts[2 * k] - pts[((k + 1) % 4) * 2]).toDouble(), (pts[2 * k + 1] - pts[((k + 1) % 4) * 2 + 1]).toDouble()) } / 4
                refineCorners(gray, pts, IntArray(4) { (0.10 * side).roundToInt().coerceIn(4, 14) })
                stickers.add(Sticker(ids.get(i, 0)[0].toInt(), pts))
            }
        }
        corners.forEach { it.release() }; ids.release()

        val cc = Mat(); val cid = Mat(); val mc = ArrayList<Mat>(); val mid = Mat()
        boardDet.detectBoard(small, cc, cid, mc, mid)
        var boardObs: BoardObs? = null
        if (!cid.empty() && cid.rows() >= 6) {
            val n = cid.rows()
            val bids = IntArray(n) { cid.get(it, 0)[0].toInt() }
            val pts = FloatArray(2 * n)
            for (i in 0 until n) { val p = cc.get(i, 0); pts[2 * i] = up(p[0], sc); pts[2 * i + 1] = up(p[1], sc) }
            // per corner: window = a tenth of the local square size (distance to the nearest other corner)
            val wins = IntArray(n) { i ->
                var best = Double.MAX_VALUE
                for (j in 0 until n) if (j != i) { val d = hypot((pts[2 * i] - pts[2 * j]).toDouble(), (pts[2 * i + 1] - pts[2 * j + 1]).toDouble()); if (d < best) best = d }
                (0.10 * best).roundToInt().coerceIn(5, 24)
            }
            refineCorners(gray, pts, wins)
            boardObs = BoardObs(bids, pts)
        }
        // a 4x4 "sticker" that sits on one of the board's 5x5 markers, or is far bigger than a 12 mm sticker
        // could be at that depth, is a misread board marker (seen on a real scan as id 189 in 30 frames)
        if (mc.isNotEmpty()) {
            val quads = mc.map { m -> val q = FloatArray(8); for (k in 0 until 4) { val p = m.get(0, k); q[2 * k] = up(p[0], sc); q[2 * k + 1] = up(p[1], sc) }; q }
            val sides = quads.map { q -> (0 until 4).sumOf { k -> hypot((q[2 * k] - q[((k + 1) % 4) * 2]).toDouble(), (q[2 * k + 1] - q[((k + 1) % 4) * 2 + 1]).toDouble()) } / 4 }.sorted()
            val boardSide = sides[sides.size / 2]
            val it = stickers.iterator()
            while (it.hasNext()) {
                val st = it.next()
                val cx = (st.corners[0] + st.corners[2] + st.corners[4] + st.corners[6]) / 4; val cy = (st.corners[1] + st.corners[3] + st.corners[5] + st.corners[7]) / 4
                val side = (0 until 4).sumOf { k -> hypot((st.corners[2 * k] - st.corners[((k + 1) % 4) * 2]).toDouble(), (st.corners[2 * k + 1] - st.corners[((k + 1) % 4) * 2 + 1]).toDouble()) } / 4
                val onMarker = quads.any { q -> insideQuad(q, cx, cy) }
                if (onMarker || side > 1.3 * boardSide * TARGET_MM / (MARKER_M * 1000)) it.remove()
            }
        }
        mc.forEach { it.release() }; mid.release()
        cc.release(); cid.release(); small.release()

        val pose = boardObs?.let { boardPose(it, w, h, focalGuessPx) }
        return Analysis(w, h, stickers, boardObs, sharp, pose, System.currentTimeMillis() - t0)
    }

    private fun insideQuad(q: FloatArray, x: Float, y: Float): Boolean {
        var pos = 0; var neg = 0
        for (k in 0 until 4) { val ax = q[2 * k]; val ay = q[2 * k + 1]; val bx = q[((k + 1) % 4) * 2]; val by = q[((k + 1) % 4) * 2 + 1]
            val cr = (bx - ax) * (y - ay) - (by - ay) * (x - ax); if (cr >= 0) pos++ else neg++ }
        return pos == 4 || neg == 4
    }

    /** Detection-image coordinate -> full-image coordinate (pixel centres: a pixel of the small image covers 1/sc pixels). */
    private fun up(v: Double, sc: Double): Float = ((v + 0.5) / sc - 0.5).toFloat()

    /**
     * Sub-pixel corners on the full-resolution image with OpenCV's cornerSubPix. wins[i] is the half-window of
     * point i; points that share a window size are refined in one call. A point that would move further than its
     * window is left where it was.
     *
     * The previous hand-written version used a fixed 5 px half-window: fine on a clean image, but phone JPEGs carry
     * sharpening and noise-reduction texture at that scale. On a real 12 MP scan its corners scattered 1.5-2 px,
     * worse than a 4x downscaled copy of the same photos.
     */
    fun refineCorners(gray: Mat, pts: FloatArray, wins: IntArray) {
        val n = pts.size / 2
        val crit = TermCriteria(TermCriteria.EPS + TermCriteria.COUNT, 60, 0.005)
        for (win in wins.distinct()) {
            val idx = (0 until n).filter { wins[it] == win && pts[2 * it] > win + 3 && pts[2 * it + 1] > win + 3 && pts[2 * it] < gray.cols() - win - 4 && pts[2 * it + 1] < gray.rows() - win - 4 }
            if (idx.isEmpty()) continue
            val m = MatOfPoint2f(*Array(idx.size) { Point(pts[2 * idx[it]].toDouble(), pts[2 * idx[it] + 1].toDouble()) })
            try {
                Imgproc.cornerSubPix(gray, m, Size(win.toDouble(), win.toDouble()), Size(-1.0, -1.0), crit)
                val out = m.toArray()
                for ((k, i) in idx.withIndex()) {
                    val dx = out[k].x - pts[2 * i]; val dy = out[k].y - pts[2 * i + 1]
                    if (!dx.isNaN() && !dy.isNaN() && hypot(dx, dy) <= 0.8 * win) { pts[2 * i] = out[k].x.toFloat(); pts[2 * i + 1] = out[k].y.toFloat() }
                }
            } catch (_: Exception) { /* keep the unrefined points */ } finally { m.release() }
        }
    }

    /** Board pose with an approximate camera (enough for slot binning; the server refines everything). */
    private fun boardPose(b: BoardObs, w: Int, h: Int, focal: Double?): Pose? {
        val f = focal ?: (0.85 * max(w, h))
        val n = b.ids.size
        val obj = MatOfPoint3f(*Array(n) { i ->
            val id = b.ids[i]
            Point3(((id % (SQUARES_X - 1)) + 1) * SQUARE_M, ((id / (SQUARES_X - 1)) + 1) * SQUARE_M, 0.0)
        })
        val img = MatOfPoint2f(*Array(n) { i -> Point(b.pts[2 * i].toDouble(), b.pts[2 * i + 1].toDouble()) })
        val K = Mat.eye(3, 3, CvType.CV_64F)
        K.put(0, 0, f); K.put(1, 1, f); K.put(0, 2, w / 2.0); K.put(1, 2, h / 2.0)
        val D = MatOfDouble(0.0, 0.0, 0.0, 0.0, 0.0)
        val rvec = Mat(); val tvec = Mat()
        val ok = Calib3d.solvePnP(obj, img, K, D, rvec, tvec, false, Calib3d.SOLVEPNP_ITERATIVE)
        if (!ok) return null
        val r = DoubleArray(3) { rvec.get(it, 0)[0] }; val t = DoubleArray(3) { tvec.get(it, 0)[0] }
        val R = Mat(); Calib3d.Rodrigues(rvec, R)
        val Rm = Array(3) { i -> DoubleArray(3) { j -> R.get(i, j)[0] } }
        // camera centre in board frame: C = -R^T t
        val C = DoubleArray(3) { j -> -(Rm[0][j] * t[0] + Rm[1][j] * t[1] + Rm[2][j] * t[2]) }
        val cx = SQUARES_X / 2.0 * SQUARE_M; val cy = SQUARES_Y / 2.0 * SQUARE_M
        val dx = C[0] - cx; val dy = C[1] - cy
        val az = ((Math.toDegrees(atan2(dx, -dy)) + 360.0) % 360.0)
        val el = Math.toDegrees(atan2(abs(C[2]), hypot(dx, dy)))
        val dist = sqrt(dx * dx + dy * dy + C[2] * C[2])
        // roll: image x-axis in the board frame (first row of R) against the x-axis an upright phone
        // aimed at the board centre would have (forward x up, with up = -z in this board frame)
        val fwd = doubleArrayOf(cx - C[0], cy - C[1], 0.0 - C[2]).let { val n = sqrt(it[0] * it[0] + it[1] * it[1] + it[2] * it[2]); doubleArrayOf(it[0] / n, it[1] / n, it[2] / n) }
        val upx = 0.0; val upy = 0.0; val upz = -1.0
        val r0x = fwd[1] * upz - fwd[2] * upy; val r0y = fwd[2] * upx - fwd[0] * upz
        val thetaExp = atan2(r0y, r0x); val thetaMeas = atan2(Rm[0][1], Rm[0][0])
        var roll = Math.toDegrees(thetaMeas - thetaExp); roll = ((roll + 540.0) % 360.0) - 180.0
        obj.release(); img.release(); K.release(); D.release(); rvec.release(); tvec.release(); R.release()
        return Pose(az, el, dist, r, t, roll)
    }
}
