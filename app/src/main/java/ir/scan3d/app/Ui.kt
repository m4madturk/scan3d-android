package ir.scan3d.app

import android.content.Context
import android.view.View
import android.view.ViewGroup
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat

/** Small helpers for edge-to-edge layouts (Android 15 draws behind the status/navigation bars). */
object Ui {
    fun dp(ctx: Context, v: Int): Int = (v * ctx.resources.displayMetrics.density + 0.5f).toInt()

    /** Same window behaviour on every Android version: content behind the bars, insets reported to our listeners. */
    fun edgeToEdge(a: android.app.Activity) = androidx.core.view.WindowCompat.setDecorFitsSystemWindows(a.window, false)

    /** Adds the system-bar / cutout insets to the view's own padding, so nothing hides under the bars. */
    fun padSystemBars(v: View, top: Boolean = true, bottom: Boolean = true) {
        val l = v.paddingLeft; val t = v.paddingTop; val r = v.paddingRight; val b = v.paddingBottom
        ViewCompat.setOnApplyWindowInsetsListener(v) { view, ins ->
            val sb = ins.getInsets(WindowInsetsCompat.Type.systemBars() or WindowInsetsCompat.Type.displayCutout())
            view.setPadding(l + sb.left, t + (if (top) sb.top else 0), r + sb.right, b + (if (bottom) sb.bottom else 0))
            ins
        }
        v.requestApplyInsets()
    }

    /** Pushes a view below the status bar by growing its top margin. */
    fun marginBelowStatusBar(v: View) {
        val lp = v.layoutParams as ViewGroup.MarginLayoutParams; val base = lp.topMargin
        ViewCompat.setOnApplyWindowInsetsListener(v) { view, ins ->
            val sb = ins.getInsets(WindowInsetsCompat.Type.systemBars() or WindowInsetsCompat.Type.displayCutout())
            (view.layoutParams as ViewGroup.MarginLayoutParams).topMargin = base + sb.top; view.requestLayout()
            ins
        }
        v.requestApplyInsets()
    }

    /** Persian digits for user-facing numbers. */
    fun fa(n: Number): String = n.toString().map { ch -> if (ch.isDigit()) ('۰' + (ch - '0')) else ch }.joinToString("")
}
