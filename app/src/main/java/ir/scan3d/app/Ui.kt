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

    /** Keeps a bottom-anchored view entirely above the navigation bar (margin, so nothing scrolls under it). */
    fun marginAboveNavBar(v: View, onInsets: ((top: Int) -> Unit)? = null) {
        val lp = v.layoutParams as ViewGroup.MarginLayoutParams; val base = lp.bottomMargin
        ViewCompat.setOnApplyWindowInsetsListener(v) { view, ins ->
            val sb = ins.getInsets(WindowInsetsCompat.Type.systemBars() or WindowInsetsCompat.Type.displayCutout())
            (view.layoutParams as ViewGroup.MarginLayoutParams).bottomMargin = base + sb.bottom; view.requestLayout()
            onInsets?.invoke(sb.top)
            ins
        }
        v.requestApplyInsets()
    }

    /** Persian digits for user-facing numbers. */
    fun fa(n: Number): String = fa(n.toString())
    fun fa(s: String): String = s.map { ch -> if (ch in '0'..'9') ('۰' + (ch - '0')) else ch }.joinToString("")

    /** A number with a fixed count of decimals in Persian digits and with the Persian decimal mark. */
    fun num(v: Double, decimals: Int = 1): String = fa(String.format(java.util.Locale.US, "%." + decimals + "f", v)).replace('.', '٫')
}
