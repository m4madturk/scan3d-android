package ir.scan3d.app

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.util.AttributeSet
import android.view.View
import kotlin.math.cos
import kotlin.math.sin

/** Draws the coverage ring (scan mode) or the frame-coverage grid (calibration mode) over the preview. */
class OverlayView(ctx: Context, attrs: AttributeSet?) : View(ctx, attrs) {
    var layout: Session.Layout? = null
    var taken: Set<String> = emptySet()
    var next: String? = null
    var current: String? = null          // slot the phone is in right now
    var calibCells: IntArray? = null     // calibration mode: 4x6 grid counts
    var stickerBoxes: List<FloatArray> = emptyList()   // preview-space quads of detected stickers
    var boardOk = false

    private val pFill = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val pStroke = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 3f }
    private val pText = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textSize = 26f; textAlign = Paint.Align.CENTER }

    override fun onDraw(c: Canvas) {
        super.onDraw(c)
        // sticker outlines
        pStroke.color = Color.argb(230, 80, 220, 120)
        for (q in stickerBoxes) {
            for (k in 0 until 4) {
                val a = k * 2; val b = ((k + 1) % 4) * 2
                c.drawLine(q[a], q[a + 1], q[b], q[b + 1], pStroke)
            }
        }
        calibCells?.let { drawGrid(c, it); return }
        val L = layout ?: return
        val r = minOf(width, height) * 0.19f
        val cx = width - r - 24f; val cy = r + 24f
        val radii = if (L.bands.size == 3) listOf(0.76f to 1.0f, 0.5f to 0.74f, 0.24f to 0.48f) else listOf(0.6f to 1.0f, 0.2f to 0.58f)
        val w = 360f / L.az
        for (b in L.bands.indices) for (a in 0 until L.az) {
            val key = "$b-$a"
            pFill.color = when {
                key == current -> Color.argb(200, 255, 200, 60)
                key in taken -> Color.argb(190, 43, 179, 170)
                key == next -> Color.argb(150, 255, 255, 255)
                else -> Color.argb(90, 255, 255, 255)
            }
            val start = a * w - w / 2 - 90f + 2f
            val outer = RectF(cx - r * radii[b].second, cy - r * radii[b].second, cx + r * radii[b].second, cy + r * radii[b].second)
            val inner = RectF(cx - r * radii[b].first, cy - r * radii[b].first, cx + r * radii[b].first, cy + r * radii[b].first)
            val p = android.graphics.Path()
            p.arcTo(outer, start, w - 4f); p.arcTo(inner, start + w - 4f, -(w - 4f)); p.close()
            c.drawPath(p, pFill)
        }
        for (a in 0 until L.az) {
            val ang = Math.toRadians((a * w - 90f).toDouble())
            c.drawText(L.clock[a], cx + (r * 1.12f * cos(ang)).toFloat(), cy + (r * 1.12f * sin(ang)).toFloat() + 9f, pText)
        }
        pFill.color = if (boardOk) Color.argb(220, 43, 179, 170) else Color.argb(220, 230, 80, 60)
        c.drawCircle(cx, cy, r * 0.12f, pFill)
    }

    private fun drawGrid(c: Canvas, cells: IntArray) {
        val gx = 4; val gy = 6
        val cw = width / gx.toFloat(); val ch = height / gy.toFloat()
        for (j in 0 until gy) for (i in 0 until gx) {
            val n = cells[j * gx + i]
            pFill.color = when { n >= 3 -> Color.argb(70, 43, 179, 170); n > 0 -> Color.argb(70, 255, 200, 60); else -> Color.argb(60, 230, 80, 60) }
            c.drawRect(i * cw + 2, j * ch + 2, (i + 1) * cw - 2, (j + 1) * ch - 2, pFill)
        }
    }
}
