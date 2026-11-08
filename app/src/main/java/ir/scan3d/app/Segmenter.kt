package ir.scan3d.app

import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import android.content.Context
import android.util.Log
import java.nio.FloatBuffer

/**
 * On-device object segmentation with the small u2netp model (4.6 MB, ONNX Runtime).
 * Run on a crop around the projected stickers: on the full photo a saliency model picks the whole
 * board; on a crop centred on the object it isolates the object itself. Gray input is replicated to
 * RGB and normalised the way rembg does (ImageNet mean/std, 320x320); the first output (d0) is
 * min-max normalised and thresholded at 0.5.
 */
class Segmenter(ctx: Context) {
    private val env: OrtEnvironment = OrtEnvironment.getEnvironment()
    private val session: OrtSession? = try {
        val bytes = ctx.assets.open("u2netp.onnx").use { it.readBytes() }
        val opts = OrtSession.SessionOptions().apply { setIntraOpNumThreads(4) }
        env.createSession(bytes, opts)
    } catch (e: Throwable) { Log.w("Scan3D", "segmenter unavailable: $e"); null }

    val available get() = session != null

    /**
     * Segments the crop [x0, x0+cw) x [y0, y0+ch) of a gray image and returns the object mask as a
     * full-size boolean array (false outside the crop), or null when the model is missing.
     */
    fun maskCrop(gray: ByteArray, w: Int, h: Int, x0: Int, y0: Int, cw: Int, ch: Int): BooleanArray? {
        val s = session ?: return null
        val n = 320
        val input = FloatArray(3 * n * n)
        for (yy in 0 until n) for (xx in 0 until n) {
            val sx = x0 + (xx + 0.5) * cw / n - 0.5; val sy = y0 + (yy + 0.5) * ch / n - 0.5
            val g = bilinear(gray, w, h, sx, sy) / 255f
            input[yy * n + xx] = (g - 0.485f) / 0.229f
            input[n * n + yy * n + xx] = (g - 0.456f) / 0.224f
            input[2 * n * n + yy * n + xx] = (g - 0.406f) / 0.225f
        }
        val out = try {
            OnnxTensor.createTensor(env, FloatBuffer.wrap(input), longArrayOf(1, 3, n.toLong(), n.toLong())).use { t ->
                s.run(mapOf(s.inputNames.first() to t)).use { r ->
                    @Suppress("UNCHECKED_CAST")
                    val v = r[0].value as Array<Array<Array<FloatArray>>>
                    v[0][0]
                }
            }
        } catch (e: Throwable) { Log.w("Scan3D", "segmenter run failed: $e"); return null }
        var mn = Float.MAX_VALUE; var mx = -Float.MAX_VALUE
        for (row in out) for (x in row) { if (x < mn) mn = x; if (x > mx) mx = x }
        val range = (mx - mn).coerceAtLeast(1e-6f)
        val mask = BooleanArray(w * h)
        for (v in 0 until ch) {
            val yy = v + y0; if (yy < 0 || yy >= h) continue
            val fy = ((v + 0.5) * n / ch - 0.5).coerceIn(0.0, (n - 1).toDouble())
            for (u in 0 until cw) {
                val xx = u + x0; if (xx < 0 || xx >= w) continue
                val fx = ((u + 0.5) * n / cw - 0.5).coerceIn(0.0, (n - 1).toDouble())
                val x0i = fx.toInt(); val y0i = fy.toInt(); val x1i = minOf(n - 1, x0i + 1); val y1i = minOf(n - 1, y0i + 1)
                val ax = (fx - x0i).toFloat(); val ay = (fy - y0i).toFloat()
                val p = (out[y0i][x0i] * (1 - ax) + out[y0i][x1i] * ax) * (1 - ay) + (out[y1i][x0i] * (1 - ax) + out[y1i][x1i] * ax) * ay
                mask[yy * w + xx] = (p - mn) / range > 0.5f
            }
        }
        return mask
    }

    private fun bilinear(g: ByteArray, w: Int, h: Int, x: Double, y: Double): Float {
        val cx = x.coerceIn(0.0, (w - 1).toDouble()); val cy = y.coerceIn(0.0, (h - 1).toDouble())
        val x0 = cx.toInt(); val y0 = cy.toInt(); val x1 = minOf(w - 1, x0 + 1); val y1 = minOf(h - 1, y0 + 1)
        val ax = (cx - x0).toFloat(); val ay = (cy - y0).toFloat()
        fun px(xx: Int, yy: Int) = (g[yy * w + xx].toInt() and 255).toFloat()
        return (px(x0, y0) * (1 - ax) + px(x1, y0) * ax) * (1 - ay) + (px(x0, y1) * (1 - ax) + px(x1, y1) * ax) * ay
    }

    fun close() { session?.close() }
}
