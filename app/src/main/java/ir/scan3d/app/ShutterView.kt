package ir.scan3d.app

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.os.SystemClock
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.min
import kotlin.math.sin

/**
 * The shutter: one big round button that also tells what the camera is doing.
 * READY white, STEADY amber and breathing (waiting for the hand to hold still), BUSY a turning arc (photo being
 * saved and measured).
 */
class ShutterView @JvmOverloads constructor(ctx: Context, attrs: AttributeSet? = null) : View(ctx, attrs) {
    enum class State { READY, STEADY, BUSY }

    var state = State.READY
        set(v) { if (field != v) { field = v; invalidate() } }

    private val d = resources.displayMetrics.density
    private val ring = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 4f * d; strokeCap = Paint.Cap.ROUND }
    private val fill = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val box = RectF()
    private var down = false
    private val white = Color.rgb(246, 248, 247); private val amber = Color.rgb(240, 180, 76); private val teal = Color.rgb(59, 194, 185); private val dim = Color.rgb(90, 104, 98)

    init { isClickable = true; isFocusable = true }

    override fun onTouchEvent(e: MotionEvent): Boolean {
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> { down = true; invalidate() }
            MotionEvent.ACTION_UP -> { if (down) { down = false; invalidate(); performClick() } }
            MotionEvent.ACTION_CANCEL -> { down = false; invalidate() }
        }
        return true
    }

    override fun performClick(): Boolean { super.performClick(); return true }

    override fun onDraw(c: Canvas) {
        super.onDraw(c)
        val cx = width / 2f; val cy = height / 2f
        val r = min(width, height) / 2f - 4f * d
        val t = SystemClock.uptimeMillis()
        when (state) {
            State.READY -> {
                ring.color = white; c.drawCircle(cx, cy, r, ring)
                fill.color = white; c.drawCircle(cx, cy, (r - 7f * d) * (if (down) 0.9f else 1f), fill)
            }
            State.STEADY -> {
                val pulse = 0.5f + 0.5f * sin(t / 160.0).toFloat()
                ring.color = amber; c.drawCircle(cx, cy, r, ring)
                fill.color = amber; c.drawCircle(cx, cy, (r - 7f * d) * (0.62f + 0.12f * pulse), fill)
                postInvalidateOnAnimation()
            }
            State.BUSY -> {
                ring.color = dim; c.drawCircle(cx, cy, r, ring)
                ring.color = teal; box.set(cx - r, cy - r, cx + r, cy + r)
                c.drawArc(box, (t % 900L) * 0.4f, 100f, false, ring)
                fill.color = dim; c.drawCircle(cx, cy, (r - 7f * d) * 0.7f, fill)
                postInvalidateOnAnimation()
            }
        }
    }
}
