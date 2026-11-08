package ir.scan3d.app

import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

/** The on-device model must load from the assets and isolate a bright object on a checkerboard crop. */
@RunWith(AndroidJUnit4::class)
class SegmenterTest {
    @Test
    fun u2netpSegmentsObjectOnCheckerboard() {
        val seg = Segmenter(ApplicationProvider.getApplicationContext())
        assertTrue("model did not load", seg.available)
        val w = 600; val h = 450
        val gray = ByteArray(w * h)
        for (y in 0 until h) for (x in 0 until w) {
            val sq = ((x / 60) + (y / 60)) % 2 == 0
            var g = if (sq) 50 else 215
            // a rounded white object in the middle with a little shading
            val dx = x - 300.0; val dy = y - 225.0
            if (dx * dx / (90.0 * 90.0) + dy * dy / (70.0 * 70.0) < 1.0) g = 225 + (dx / 30).toInt()
            gray[y * w + x] = g.coerceIn(0, 255).toByte()
        }
        val t0 = System.currentTimeMillis()
        val mask = seg.maskCrop(gray, w, h, 120, 60, 360, 330)!!
        val ms = System.currentTimeMillis() - t0
        var inside = 0; var total = 0; var outsideHits = 0; var outsideTotal = 0
        for (y in 60 until 390) for (x in 120 until 480) {
            val dx = x - 300.0; val dy = y - 225.0
            val obj = dx * dx / (90.0 * 90.0) + dy * dy / (70.0 * 70.0) < 1.0
            val far = dx * dx / (110.0 * 110.0) + dy * dy / (90.0 * 90.0) > 1.0
            if (obj) { total++; if (mask[y * w + x]) inside++ } else if (far) { outsideTotal++; if (mask[y * w + x]) outsideHits++ }
        }
        android.util.Log.i("Scan3D", "segmenter: %d ms, object covered %.1f%%, background wrongly %.1f%%".format(ms, 100.0 * inside / total, 100.0 * outsideHits / outsideTotal))
        assertTrue("object coverage %.1f%%".format(100.0 * inside / total), inside > 0.85 * total)
        assertTrue("background false positives %.1f%%".format(100.0 * outsideHits / outsideTotal), outsideHits < 0.15 * outsideTotal)
        seg.close()
    }
}
