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

/** Live preview result: sticker ids, number of board corners and (if the board is seen) the pose. */
data class Live(val ids: List<Int>, val nBoard: Int, val pose: Pose?)

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

    /** Preview-resolution detection plus a quick board pose (approximate focal length, no distortion). */
    fun live(gray: Mat, focalPx: Double?): Live {
        val corners = ArrayList<Mat>(); val ids = Mat()
        targetDet.detectMarkers(gray, corners, ids)
        val idList = ArrayList<Int>()
        if (!ids.empty()) for (i in 0 until ids.rows()) idList.add(ids.get(i, 0)[0].toInt())
        corners.forEach { it.release() }; ids.release()
        val cc = Mat(); val cid = Mat()
        boardDet.detectBoard(gray, cc, cid)
        val nBoard = if (cid.empty()) 0 else cid.rows()
        var pose: Pose? = null
        if (nBoard >= 6) {
            val bids = IntArray(nBoard) { cid.get(it, 0)[0].toInt() }
            val pts = FloatArray(2 * nBoard); for (i in 0 until nBoard) { val v = cc.get(i, 0); pts[2 * i] = v[0].toFloat(); pts[2 * i + 1] = v[1].toFloat() }
            pose = try { boardPose(BoardObs(bids, pts), gray.cols(), gray.rows(), focalPx) } catch (_: Exception) { null }
        }
        cc.release(); cid.release()
        return Live(idList, nBoard, pose)
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
        lap.release()

        val corners = ArrayList<Mat>(); val ids = Mat()
        targetDet.detectMarkers(small, corners, ids)
        val stickers = ArrayList<Sticker>()
        if (!ids.empty()) {
            for (i in 0 until min(ids.rows(), corners.size)) {
                val c = corners[i]
                val pts = FloatArray(8)
                for (k in 0 until 4) { val p = c.get(0, k); pts[2 * k] = (p[0] / sc).toFloat(); pts[2 * k + 1] = (p[1] / sc).toFloat() }
                refineFull(gray, pts)
                stickers.add(Sticker(ids.get(i, 0)[0].toInt(), pts))
            }
        }
        corners.forEach { it.release() }; ids.release()

        val cc = Mat(); val cid = Mat()
        boardDet.detectBoard(small, cc, cid)
        var boardObs: BoardObs? = null
        if (!cid.empty() && cid.rows() >= 6) {
            val n = cid.rows()
            val bids = IntArray(n) { cid.get(it, 0)[0].toInt() }
            val pts = FloatArray(2 * n)
            for (i in 0 until n) { val p = cc.get(i, 0); pts[2 * i] = (p[0] / sc).toFloat(); pts[2 * i + 1] = (p[1] / sc).toFloat() }
            refineFull(gray, pts)
            boardObs = BoardObs(bids, pts)
        }
        cc.release(); cid.release(); small.release()

        val pose = boardObs?.let { boardPose(it, w, h, focalGuessPx) }
        return Analysis(w, h, stickers, boardObs, sharp, pose, System.currentTimeMillis() - t0)
    }

    /** Port of OpenCV cornerSubPix (gradient-orthogonality), on the full-resolution image. */
    fun refineFull(gray: Mat, pts: FloatArray, win: Int = 5, maxIter: Int = 40, eps: Double = 0.01) {
        val w = gray.cols(); val h = gray.rows()
        val buf = ByteArray(w * h); gray.get(0, 0, buf)
        fun px(x: Double, y: Double): Double {
            val x0 = x.toInt(); val y0 = y.toInt()
            if (x < 0 || y < 0 || x0 + 1 >= w || y0 + 1 >= h) return -1.0
            val fx = x - x0; val fy = y - y0; val o = y0 * w + x0
            val a = (buf[o].toInt() and 255); val b = (buf[o + 1].toInt() and 255)
            val c = (buf[o + w].toInt() and 255); val d = (buf[o + w + 1].toInt() and 255)
            return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy
        }
        var k = 0
        while (k < pts.size) {
            val x0 = pts[k].toDouble(); val y0 = pts[k + 1].toDouble()
            var x = x0; var y = y0; var ok = true
            for (it in 0 until maxIter) {
                var a = 0.0; var b = 0.0; var c = 0.0; var b1 = 0.0; var b2 = 0.0
                loop@ for (j in -win..win) for (i in -win..win) {
                    val sx = x + i; val sy = y + j
                    val l = px(sx - 1, sy); val r = px(sx + 1, sy); val u = px(sx, sy - 1); val d = px(sx, sy + 1)
                    if (l < 0 || r < 0 || u < 0 || d < 0) { ok = false; break@loop }
                    val gx = (r - l) * 0.5; val gy = (d - u) * 0.5
                    val wgt = Math.exp(-(i * i + j * j).toDouble() / (win * win))
                    val gxx = gx * gx * wgt; val gxy = gx * gy * wgt; val gyy = gy * gy * wgt
                    a += gxx; b += gxy; c += gyy; b1 += gxx * sx + gxy * sy; b2 += gxy * sx + gyy * sy
                }
                if (!ok) break
                val det = a * c - b * b
                if (abs(det) < 1e-9) { ok = false; break }
                val nx = (c * b1 - b * b2) / det; val ny = (a * b2 - b * b1) / det
                val step = hypot(nx - x, ny - y)
                x = nx; y = ny
                if (hypot(x - x0, y - y0) > win) { ok = false; break }
                if (step < eps) break
            }
            if (ok) { pts[k] = x.toFloat(); pts[k + 1] = y.toFloat() }
            k += 2
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
