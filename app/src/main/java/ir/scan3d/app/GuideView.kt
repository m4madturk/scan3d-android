package ir.scan3d.app

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.util.AttributeSet
import android.view.View
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Animated 3D illustration of how to hold the phone: the ChArUco board on the table, the object with
 * its stickers, the 18 capture positions as dots, the phone where it is now (white) and where it
 * should go (teal ghost). Pure Canvas, orthographic projection, no GL.
 *
 * Scene frame (mm): board centre at the origin, x right, y towards 12 o'clock, z up.
 * Azimuth = clock direction (0 = 12 o'clock, 90 = 3 o'clock), elevation above the board plane.
 */
class GuideView @JvmOverloads constructor(ctx: Context, attrs: AttributeSet? = null) : View(ctx, attrs) {
    /** A phone pose on the orbit around the aim point; roll 90 = phone held sideways. */
    data class PhonePose(val az: Double, val el: Double, val dist: Double, val roll: Double = 0.0,
                         val aimX: Double = 0.0, val aimY: Double = 0.0, val aimZ: Double = 15.0)

    enum class Mode { SCAN, CALIB }

    var mode = Mode.SCAN
    var showObject = true
    var target: PhonePose? = null            // teal ghost
    var live: PhonePose? = null              // measured pose (smoothed into `shown`)
    var liveFresh = false                    // false -> drawn faded (board not seen right now)
    var slots: Map<String, Int> = emptyMap() // "band-az" -> 0 missing, 1 taken, 2 next
    var slotAz = 6
    var bandEl = listOf(22.0, 47.0, 75.0)
    var orbitMm = 320.0
    var keyframes: List<PhonePose> = emptyList()   // tutorial animation; overrides live
    var segmentMs = 1500L
    var holdMs = 500L
    var viewAz = 205.0
    var viewEl = 34.0
    var caption: String? = null
    var showClock = true

    private var shown: PhonePose? = null
    private var animStart = 0L

    private val pBoardDark = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(44, 52, 49); style = Paint.Style.FILL }
    private val pBoardLight = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(214, 222, 218); style = Paint.Style.FILL }
    private val pBoardEdge = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(240, 244, 242); style = Paint.Style.STROKE; strokeWidth = 2f }
    private val pFill = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val pStroke = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 2f; strokeJoin = Paint.Join.ROUND }
    private val pText = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textAlign = Paint.Align.CENTER }
    private val pDash = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 2f; pathEffect = DashPathEffect(floatArrayOf(8f, 6f), 0f) }
    private val path = Path()

    // ---- projection ----
    private var cx0 = 0f; private var cy0 = 0f; private var scale = 1f
    private val fwd = DoubleArray(3); private val right = DoubleArray(3); private val upv = DoubleArray(3)

    private fun setupView() {
        val a = Math.toRadians(viewAz); val e = Math.toRadians(viewEl)
        val vx = sin(a) * cos(e); val vy = cos(a) * cos(e); val vz = sin(e)     // viewer direction from the origin
        fwd[0] = -vx; fwd[1] = -vy; fwd[2] = -vz
        // right = fwd x up
        right[0] = fwd[1] * 1 - fwd[2] * 0; right[1] = fwd[2] * 0 - fwd[0] * 1; right[2] = fwd[0] * 0 - fwd[1] * 0
        val rn = sqrt(right[0] * right[0] + right[1] * right[1] + right[2] * right[2]); for (i in 0..2) right[i] /= rn
        // up = right x fwd
        upv[0] = right[1] * fwd[2] - right[2] * fwd[1]; upv[1] = right[2] * fwd[0] - right[0] * fwd[2]; upv[2] = right[0] * fwd[1] - right[1] * fwd[0]
        val reach = if (mode == Mode.SCAN) orbitMm + 120 else 420.0
        scale = (min(width, height) * 0.5f / reach.toFloat()) * 0.96f
        cx0 = width / 2f; cy0 = height / 2f + min(width, height) * 0.06f
    }

    private fun px(x: Double, y: Double, z: Double): FloatArray {
        val sx = cx0 + scale * (x * right[0] + y * right[1] + z * right[2]).toFloat()
        val sy = cy0 - scale * (x * upv[0] + y * upv[1] + z * upv[2]).toFloat()
        val d = (x * fwd[0] + y * fwd[1] + z * fwd[2]).toFloat()
        return floatArrayOf(sx, sy, d)
    }

    // ---- scene helpers ----
    private class Face(val pts: Array<DoubleArray>, val color: Int, val edge: Int?, val depth: Double)

    private fun face(color: Int, edge: Int?, vararg p: DoubleArray): Face {
        var d = 0.0; for (q in p) d += q[0] * fwd[0] + q[1] * fwd[1] + q[2] * fwd[2]
        return Face(arrayOf(*p), color, edge, d / p.size)
    }

    private fun v(x: Double, y: Double, z: Double) = doubleArrayOf(x, y, z)

    private fun phoneFaces(p: PhonePose, body: Int, screen: Int, edge: Int, ghost: Boolean): List<Face> {
        val a = Math.toRadians(p.az); val e = Math.toRadians(p.el)
        val dir = doubleArrayOf(sin(a) * cos(e), cos(a) * cos(e), sin(e))
        val P = doubleArrayOf(p.aimX + p.dist * dir[0], p.aimY + p.dist * dir[1], p.aimZ + p.dist * dir[2])
        val f = doubleArrayOf(-dir[0], -dir[1], -dir[2])                                // optical axis towards the aim point
        // phone "up": world up projected onto the sensor plane (or +y when looking straight down)
        var u = doubleArrayOf(0 - f[2] * f[0], 0 - f[2] * f[1], 1 - f[2] * f[2])
        var un = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2])
        if (un < 1e-3) { u = doubleArrayOf(0 - f[1] * f[0], 1 - f[1] * f[1], 0 - f[1] * f[2]); un = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]) }
        for (i in 0..2) u[i] /= un
        var r = doubleArrayOf(f[1] * u[2] - f[2] * u[1], f[2] * u[0] - f[0] * u[2], f[0] * u[1] - f[1] * u[0])
        val ro = Math.toRadians(p.roll)
        val u2 = DoubleArray(3) { cos(ro) * u[it] + sin(ro) * r[it] }
        val r2 = DoubleArray(3) { -sin(ro) * u[it] + cos(ro) * r[it] }
        u = u2; r = r2
        val hw = 36.0; val hh = 76.0; val th = 9.0
        fun c(sr: Double, su: Double, sf: Double) = doubleArrayOf(P[0] + sr * r[0] + su * u[0] + sf * f[0], P[1] + sr * r[1] + su * u[1] + sf * f[1], P[2] + sr * r[2] + su * u[2] + sf * f[2])
        val b00 = c(-hw, -hh, 0.0); val b10 = c(hw, -hh, 0.0); val b11 = c(hw, hh, 0.0); val b01 = c(-hw, hh, 0.0)   // back (lens side)
        val s00 = c(-hw, -hh, -th); val s10 = c(hw, -hh, -th); val s11 = c(hw, hh, -th); val s01 = c(-hw, hh, -th)   // screen side
        val out = ArrayList<Face>()
        out += face(body, edge, b00, b10, b11, b01)
        out += face(screen, edge, s00, s10, s11, s01)
        out += face(body, edge, b00, b10, s10, s00); out += face(body, edge, b01, b11, s11, s01)
        out += face(body, edge, b00, b01, s01, s00); out += face(body, edge, b10, b11, s11, s10)
        // lens: small square on the back, top-left corner
        val l = 9.0; val lc = c(-hw + 16, hh - 18, 0.6)
        val lx = DoubleArray(3) { r[it] * l }; val ly = DoubleArray(3) { u[it] * l }
        out += face(if (ghost) Color.argb(160, 10, 40, 38) else Color.rgb(12, 18, 16), null,
            doubleArrayOf(lc[0] - lx[0] - ly[0], lc[1] - lx[1] - ly[1], lc[2] - lx[2] - ly[2]), doubleArrayOf(lc[0] + lx[0] - ly[0], lc[1] + lx[1] - ly[1], lc[2] + lx[2] - ly[2]),
            doubleArrayOf(lc[0] + lx[0] + ly[0], lc[1] + lx[1] + ly[1], lc[2] + lx[2] + ly[2]), doubleArrayOf(lc[0] - lx[0] + ly[0], lc[1] - lx[1] + ly[1], lc[2] - lx[2] + ly[2]))
        return out
    }

    private fun boxFaces(cx: Double, cy: Double, sx: Double, sy: Double, sz: Double): List<Face> {
        val x0 = cx - sx / 2; val x1 = cx + sx / 2; val y0 = cy - sy / 2; val y1 = cy + sy / 2
        val top = Color.rgb(70, 196, 188); val side = Color.rgb(42, 150, 143); val side2 = Color.rgb(30, 118, 112); val edge = Color.rgb(10, 60, 56)
        return listOf(
            face(top, edge, v(x0, y0, sz), v(x1, y0, sz), v(x1, y1, sz), v(x0, y1, sz)),
            face(side, edge, v(x0, y0, 0.0), v(x1, y0, 0.0), v(x1, y0, sz), v(x0, y0, sz)),
            face(side, edge, v(x0, y1, 0.0), v(x1, y1, 0.0), v(x1, y1, sz), v(x0, y1, sz)),
            face(side2, edge, v(x0, y0, 0.0), v(x0, y1, 0.0), v(x0, y1, sz), v(x0, y0, sz)),
            face(side2, edge, v(x1, y0, 0.0), v(x1, y1, 0.0), v(x1, y1, sz), v(x1, y0, sz)))
    }

    private fun drawFace(c: Canvas, f: Face) {
        path.reset()
        for ((i, p) in f.pts.withIndex()) { val q = px(p[0], p[1], p[2]); if (i == 0) path.moveTo(q[0], q[1]) else path.lineTo(q[0], q[1]) }
        path.close()
        pFill.color = f.color; c.drawPath(path, pFill)
        if (f.edge != null) { pStroke.color = f.edge; pStroke.strokeWidth = 1.5f * density(); c.drawPath(path, pStroke) }
    }

    private fun density() = resources.displayMetrics.density

    // ---- animation / smoothing ----
    private fun currentPose(now: Long): PhonePose? {
        if (keyframes.isNotEmpty()) {
            if (keyframes.size == 1) return keyframes[0]
            if (animStart == 0L) animStart = now
            val seg = segmentMs + holdMs; val total = seg * keyframes.size
            val t = (now - animStart) % total
            val i = (t / seg).toInt(); val u = (t % seg).toDouble()
            val a = keyframes[i]; val b = keyframes[(i + 1) % keyframes.size]
            val s = if (u >= segmentMs) 1.0 else { val x = u / segmentMs; x * x * (3 - 2 * x) }
            return lerp(a, b, s)
        }
        val l = live ?: return shown
        val s = shown ?: l
        shown = lerp(s, l, 0.25); return shown
    }

    private fun lerp(a: PhonePose, b: PhonePose, s: Double): PhonePose {
        var daz = ((b.az - a.az + 540) % 360) - 180
        return PhonePose(a.az + daz * s, a.el + (b.el - a.el) * s, a.dist + (b.dist - a.dist) * s, a.roll + (b.roll - a.roll) * s,
            a.aimX + (b.aimX - a.aimX) * s, a.aimY + (b.aimY - a.aimY) * s, a.aimZ + (b.aimZ - a.aimZ) * s)
    }

    /** wrap_content height -> a square (capped at ~45 % of the screen height). */
    override fun onMeasure(wSpec: Int, hSpec: Int) {
        val w = MeasureSpec.getSize(wSpec)
        if (MeasureSpec.getMode(hSpec) == MeasureSpec.EXACTLY) { super.onMeasure(wSpec, hSpec); return }
        val cap = (resources.displayMetrics.heightPixels * 0.45f).toInt()
        val h = min(w, cap)
        setMeasuredDimension(w, h)
    }

    override fun onDraw(c: Canvas) {
        super.onDraw(c)
        if (width == 0 || height == 0) return
        setupView()
        val d = density()
        // background
        pFill.color = Color.rgb(15, 21, 19); c.drawRoundRect(RectF(0f, 0f, width.toFloat(), height.toFloat()), 12 * d, 12 * d, pFill)

        // board 7 x 10 squares of 25 mm, centred
        val bw = 175.0; val bh = 250.0; val sq = 25.0
        for (i in 0 until 7) for (j in 0 until 10) {
            val x0 = -bw / 2 + i * sq; val y0 = -bh / 2 + j * sq
            val p0 = px(x0, y0, 0.0); val p1 = px(x0 + sq, y0, 0.0); val p2 = px(x0 + sq, y0 + sq, 0.0); val p3 = px(x0, y0 + sq, 0.0)
            path.reset(); path.moveTo(p0[0], p0[1]); path.lineTo(p1[0], p1[1]); path.lineTo(p2[0], p2[1]); path.lineTo(p3[0], p3[1]); path.close()
            c.drawPath(path, if ((i + j) % 2 == 0) pBoardDark else pBoardLight)
        }
        run { val p0 = px(-bw / 2, -bh / 2, 0.0); val p1 = px(bw / 2, -bh / 2, 0.0); val p2 = px(bw / 2, bh / 2, 0.0); val p3 = px(-bw / 2, bh / 2, 0.0)
            path.reset(); path.moveTo(p0[0], p0[1]); path.lineTo(p1[0], p1[1]); path.lineTo(p2[0], p2[1]); path.lineTo(p3[0], p3[1]); path.close()
            pBoardEdge.strokeWidth = 1.5f * d; c.drawPath(path, pBoardEdge) }

        val now = System.currentTimeMillis()
        val cur = currentPose(now)
        val faces = ArrayList<Face>()
        if (showObject && mode == Mode.SCAN) faces += boxFaces(0.0, 0.0, 62.0, 42.0, 30.0)

        // orbit slots (scan) — dots on the orbit sphere around the aim point
        if (mode == Mode.SCAN && slots.isNotEmpty()) {
            for ((key, st) in slots) {
                val (b, a) = key.split("-").map { it.toInt() }
                if (b >= bandEl.size) continue
                val az = Math.toRadians(a * 360.0 / slotAz); val el = Math.toRadians(bandEl[b])
                val p = px(orbitMm * sin(az) * cos(el), orbitMm * cos(az) * cos(el), 15.0 + orbitMm * sin(el))
                val r = (if (st == 2) 7f else 5f) * d
                pFill.color = when (st) { 1 -> Color.rgb(59, 194, 185); 2 -> Color.rgb(255, 204, 64); else -> Color.argb(160, 120, 132, 126) }
                c.drawCircle(p[0], p[1], r, pFill)
                if (st == 2) { pStroke.color = Color.rgb(255, 204, 64); pStroke.strokeWidth = 2f * d; c.drawCircle(p[0], p[1], r + 4 * d, pStroke) }
            }
        }
        // clock numerals on the table around the board
        if (showClock && mode == Mode.SCAN) {
            pText.textSize = 11f * d; pText.color = Color.rgb(170, 184, 178)
            for ((k, lab) in listOf("۱۲", "۳", "۶", "۹").withIndex()) {
                val az = Math.toRadians(k * 90.0); val p = px(190.0 * sin(az), 190.0 * cos(az), 0.0)
                c.drawText(lab, p[0], p[1] + 4 * d, pText)
            }
        }

        // phones
        target?.let { faces += phoneFaces(it, Color.argb(120, 59, 194, 185), Color.argb(120, 20, 90, 86), Color.argb(200, 59, 194, 185), true) }
        if (cur != null) {
            val alpha = if (keyframes.isNotEmpty() || liveFresh) 255 else 110
            faces += phoneFaces(cur, Color.argb(alpha, 229, 236, 232), Color.argb(alpha, 24, 32, 30), Color.argb(alpha, 120, 132, 126), false)
        }
        faces.sortByDescending { it.depth }       // far first (depth grows away from the viewer)
        for (f in faces) drawFace(c, f)

        // sight line from the lens to the aim point, and the move arrow
        if (cur != null) {
            val a = Math.toRadians(cur.az); val e = Math.toRadians(cur.el)
            val P = px(cur.aimX + cur.dist * sin(a) * cos(e), cur.aimY + cur.dist * cos(a) * cos(e), cur.aimZ + cur.dist * sin(e))
            val A = px(cur.aimX, cur.aimY, cur.aimZ)
            pDash.color = Color.argb(150, 229, 236, 232); pDash.strokeWidth = 1.5f * d
            path.reset(); path.moveTo(P[0], P[1]); path.lineTo(A[0], A[1]); c.drawPath(path, pDash)
            target?.let { t ->
                val ta = Math.toRadians(t.az); val te = Math.toRadians(t.el)
                val T = px(t.aimX + t.dist * sin(ta) * cos(te), t.aimY + t.dist * cos(ta) * cos(te), t.aimZ + t.dist * sin(te))
                val far = abs(((t.az - cur.az + 540) % 360) - 180) > 12 || abs(t.el - cur.el) > 8 || abs(t.dist - cur.dist) > 60
                if (far && keyframes.isEmpty()) {
                    pStroke.color = Color.rgb(255, 204, 64); pStroke.strokeWidth = 3f * d
                    val mx = (P[0] + T[0]) / 2; val my = (P[1] + T[1]) / 2 - 40 * d
                    path.reset(); path.moveTo(P[0], P[1]); path.quadTo(mx, my, T[0], T[1]); c.drawPath(path, pStroke)
                    // arrow head
                    val dx = T[0] - mx; val dy = T[1] - my; val n = sqrt(dx * dx + dy * dy).coerceAtLeast(1f)
                    val ux = dx / n; val uy = dy / n; val s = 9 * d
                    pFill.color = Color.rgb(255, 204, 64)
                    path.reset(); path.moveTo(T[0], T[1]); path.lineTo(T[0] - s * ux + s * 0.6f * uy, T[1] - s * uy - s * 0.6f * ux); path.lineTo(T[0] - s * ux - s * 0.6f * uy, T[1] - s * uy + s * 0.6f * ux); path.close()
                    c.drawPath(path, pFill)
                }
            }
        }
        caption?.let { pText.textSize = 12f * d; pText.color = Color.rgb(229, 236, 232); c.drawText(it, width / 2f, height - 8 * d, pText) }
        if (keyframes.size > 1 || live != null) postInvalidateOnAnimation()
    }

    /** Convenience: slot ring for the scan layout. */
    fun setSlots(layout: Session.Layout?, taken: Set<String>, next: String?) {
        if (layout == null) { slots = emptyMap(); return }
        slotAz = layout.az; bandEl = layout.bands.map { (it.first + it.second) / 2 }
        val m = HashMap<String, Int>()
        for (b in layout.bands.indices) for (a in 0 until layout.az) { val k = "$b-$a"; m[k] = if (k == next) 2 else if (k in taken) 1 else 0 }
        slots = m
    }

    companion object {
        /** Target pose for a scan slot. */
        fun slotPose(layout: Session.Layout, key: String, dist: Double = 320.0): PhonePose {
            val (b, a) = key.split("-").map { it.toInt() }
            val el = (layout.bands[b].first + layout.bands[b].second) / 2
            return PhonePose(a * 360.0 / layout.az, el, dist)
        }

        /** Target pose for a calibration step (see CalibPlanner.Target). */
        fun calibPose(t: CalibPlanner.Target): PhonePose = when (t.kind) {
            "front" -> PhonePose(180.0, 89.0, 300.0, 0.0, 0.0, 0.0, 0.0)
            "close" -> PhonePose(180.0, 89.0, 230.0, 0.0, 0.0, 0.0, 0.0)
            "tilt" -> when (t.dir) { 1 -> PhonePose(180.0, 50.0, 280.0, 0.0, 0.0, 0.0, 0.0); 3 -> PhonePose(0.0, 50.0, 280.0, 0.0, 0.0, 0.0, 0.0)
                                     2 -> PhonePose(270.0, 50.0, 280.0, 0.0, 0.0, 0.0, 0.0); else -> PhonePose(90.0, 50.0, 280.0, 0.0, 0.0, 0.0, 0.0) }
            "roll" -> PhonePose(180.0, 85.0, 300.0, 90.0, 0.0, 0.0, 0.0)
            "cover" -> { val cx = t.cell % 4; val cy = t.cell / 4; PhonePose(180.0, 89.0, 300.0, 0.0, (1.5 - cx) * 45.0, (cy - 2.5) * 40.0, 0.0) }
            "more" -> PhonePose(140.0, 65.0, 360.0, 0.0, 0.0, 0.0, 0.0)
            else -> PhonePose(180.0, 89.0, 300.0, 0.0, 0.0, 0.0, 0.0)
        }
    }
}
