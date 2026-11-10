package ir.scan3d.app

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.util.AttributeSet
import android.view.View
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

/**
 * Drawn over the camera preview: what the detector sees right now (outline of the printed sheet, a frame
 * around every sticker), a small coverage ring (scan) or the frame-coverage grid (calibration).
 * Everything is placed relative to the preview IMAGE, which sits letter-boxed inside this view.
 */
class OverlayView(ctx: Context, attrs: AttributeSet?) : View(ctx, attrs) {
    var layout: Session.Layout? = null
    var taken: Set<String> = emptySet()
    var next: String? = null
    var current: String? = null          // slot the phone is in right now
    var calibCells: IntArray? = null     // calibration mode: 4x6 grid counts
    var boardOk = false
    var live: Live? = null               // detections in preview-image pixels
    var topInset = 0f                    // kept for callers; the image rect already clears the bars

    private val d = resources.displayMetrics.density
    private val pFill = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val pStroke = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeJoin = Paint.Join.ROUND; strokeCap = Paint.Cap.ROUND }
    private val path = Path()
    private val img = RectF()

    private val teal = Color.rgb(59, 194, 185); private val green = Color.rgb(96, 226, 138); private val amber = Color.rgb(240, 180, 76); private val red = Color.rgb(229, 96, 77)

    /** Where the preview image lies inside the view (PreviewView scale type fitCenter). */
    private fun imageRect() {
        val l = live
        val aspect = if (l != null && l.w > 0 && l.h > 0) l.w.toFloat() / l.h else 0.75f
        var w = width.toFloat(); var h = w / aspect
        if (h > height) { h = height.toFloat(); w = h * aspect }
        img.set((width - w) / 2f, (height - h) / 2f, (width + w) / 2f, (height + h) / 2f)
    }

    override fun onDraw(c: Canvas) {
        super.onDraw(c)
        if (width == 0 || height == 0) return
        imageRect()
        // ---- live detections ----
        live?.let { l ->
            if (l.w > 0 && l.h > 0) {
                val sx = img.width() / l.w; val sy = img.height() / l.h
                l.outline?.let { o ->
                    path.reset()
                    for (k in 0 until 4) { val x = img.left + o[2 * k] * sx; val y = img.top + o[2 * k + 1] * sy; if (k == 0) path.moveTo(x, y) else path.lineTo(x, y) }
                    path.close()
                    c.save(); c.clipRect(img)
                    pStroke.color = teal; pStroke.alpha = 220; pStroke.strokeWidth = 2f * d; c.drawPath(path, pStroke)
                    c.restore()
                }
                pStroke.color = green; pStroke.alpha = 255; pStroke.strokeWidth = 2f * d
                for (q in l.quads) {
                    path.reset()
                    for (k in 0 until 4) { val x = img.left + q[2 * k] * sx; val y = img.top + q[2 * k + 1] * sy; if (k == 0) path.moveTo(x, y) else path.lineTo(x, y) }
                    path.close(); c.drawPath(path, pStroke)
                }
            }
        }
        calibCells?.let { drawGrid(c, it); return }
        val L = layout ?: return
        drawRing(c, L)
    }

    /** Small coverage ring in the top corner of the image: one sector per capture position, filled when taken. */
    private fun drawRing(c: Canvas, L: Session.Layout) {
        val r = min(30f * d, min(img.width(), img.height()) * 0.11f)
        val cx = img.right - r - 10f * d; val cy = img.top + r + 10f * d
        pFill.color = Color.argb(120, 0, 0, 0); c.drawCircle(cx, cy, r + 5f * d, pFill)
        val radii = if (L.bands.size == 3) listOf(0.74f to 1.0f, 0.46f to 0.70f, 0.18f to 0.42f) else listOf(0.60f to 1.0f, 0.20f to 0.56f)
        val w = 360f / L.az; val gap = 5f
        for (b in L.bands.indices) for (a in 0 until L.az) {
            val key = "$b-$a"
            pFill.color = when {
                key == current -> Color.argb(255, 240, 180, 76)
                key in taken -> Color.argb(235, 59, 194, 185)
                key == next -> Color.argb(210, 255, 255, 255)
                else -> Color.argb(70, 255, 255, 255)
            }
            val start = a * w - w / 2 - 90f + gap / 2
            val ro = r * radii[b].second; val ri = r * radii[b].first
            path.reset()
            path.arcTo(RectF(cx - ro, cy - ro, cx + ro, cy + ro), start, w - gap)
            path.arcTo(RectF(cx - ri, cy - ri, cx + ri, cy + ri), start + w - gap, -(w - gap))
            path.close()
            c.drawPath(path, pFill)
        }
        // 12 o'clock tick and the board lamp in the middle
        pStroke.color = Color.WHITE; pStroke.alpha = 200; pStroke.strokeWidth = 1.5f * d
        c.drawLine(cx, cy - r - 4f * d, cx, cy - r - 1f * d, pStroke)
        pFill.color = if (boardOk) teal else red
        c.drawCircle(cx, cy, r * 0.10f, pFill)
    }

    /** Calibration: which parts of the frame have seen board corners. Thin lines; only the missing cells are tinted. */
    private fun drawGrid(c: Canvas, cells: IntArray) {
        val gx = 4; val gy = 6
        val cw = img.width() / gx; val ch = img.height() / gy
        for (j in 0 until gy) for (i in 0 until gx) {
            val n = cells[j * gx + i]
            val l = img.left + i * cw; val t = img.top + j * ch
            if (n < 3) {     // still to cover: an amber ring in the middle of the cell (filled once the board has touched it)
                pStroke.color = amber; pStroke.alpha = 230; pStroke.strokeWidth = 2f * d
                c.drawCircle(l + cw / 2, t + ch / 2, 6f * d, pStroke)
                if (n > 0) { pFill.color = amber; pFill.alpha = 230; c.drawCircle(l + cw / 2, t + ch / 2, 2.5f * d, pFill) }
            } else {         // covered: a small tick in the corner, nothing over the picture
                pStroke.color = teal; pStroke.alpha = 230; pStroke.strokeWidth = 2f * d
                val x = l + cw - 14f * d; val y = t + 12f * d
                c.drawLine(x - 4f * d, y, x - 1f * d, y + 3f * d, pStroke); c.drawLine(x - 1f * d, y + 3f * d, x + 5f * d, y - 4f * d, pStroke)
            }
        }
        pStroke.color = Color.WHITE; pStroke.alpha = 60; pStroke.strokeWidth = 1f * d
        for (i in 1 until gx) c.drawLine(img.left + i * cw, img.top, img.left + i * cw, img.bottom, pStroke)
        for (j in 1 until gy) c.drawLine(img.left, img.top + j * ch, img.right, img.top + j * ch, pStroke)
    }
}
