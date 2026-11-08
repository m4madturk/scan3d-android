package ir.scan3d.app

import android.os.Bundle
import android.view.View
import androidx.appcompat.app.AppCompatActivity
import ir.scan3d.app.GuideView.PhonePose
import ir.scan3d.app.databinding.ActivityTutorialBinding

/** Step-by-step tutorial: each page is an animated 3D illustration plus a short explanation. */
class TutorialActivity : AppCompatActivity() {
    private lateinit var b: ActivityTutorialBinding
    private var page = 0

    private class Page(val title: Int, val body: Int, val mode: GuideView.Mode, val obj: Boolean, val frames: List<PhonePose>, val segMs: Long = 1500, val slots: Boolean = false)

    private val pages by lazy {
        val orbit = ArrayList<PhonePose>()
        for (el in listOf(22.0, 47.0, 75.0)) for (a in 0 until 6) orbit += PhonePose(a * 60.0, el, 320.0)
        listOf(
            Page(R.string.t1_title, R.string.t1_body, GuideView.Mode.SCAN, true, listOf(PhonePose(150.0, 40.0, 320.0))),
            Page(R.string.t2_title, R.string.t2_body, GuideView.Mode.SCAN, true, listOf(PhonePose(200.0, 30.0, 300.0), PhonePose(130.0, 35.0, 300.0)), 2200),
            Page(R.string.t3_title, R.string.t3_body, GuideView.Mode.SCAN, true, orbit, 700, true),
            Page(R.string.t4_title, R.string.t4_body, GuideView.Mode.SCAN, true, listOf(PhonePose(180.0, 18.0, 320.0), PhonePose(240.0, 18.0, 320.0), PhonePose(300.0, 18.0, 320.0)), 1300),
            Page(R.string.t5_title, R.string.t5_body, GuideView.Mode.SCAN, true, listOf(PhonePose(200.0, 45.0, 320.0, 0.0), PhonePose(200.0, 45.0, 320.0, 90.0)), 1800),
            Page(R.string.t6_title, R.string.t6_body, GuideView.Mode.SCAN, true, listOf(PhonePose(210.0, 50.0, 300.0))),
            Page(R.string.t7_title, R.string.t7_body, GuideView.Mode.CALIB, false, listOf(
                PhonePose(180.0, 89.0, 260.0, 0.0, 0.0, 0.0, 0.0), PhonePose(180.0, 50.0, 280.0, 0.0, 0.0, 0.0, 0.0), PhonePose(0.0, 50.0, 280.0, 0.0, 0.0, 0.0, 0.0),
                PhonePose(270.0, 50.0, 280.0, 0.0, 0.0, 0.0, 0.0), PhonePose(90.0, 50.0, 280.0, 0.0, 0.0, 0.0, 0.0), PhonePose(180.0, 85.0, 300.0, 90.0, 0.0, 0.0, 0.0)), 1300),
            Page(R.string.t8_title, R.string.t8_body, GuideView.Mode.CALIB, false, listOf(
                PhonePose(180.0, 89.0, 300.0, 0.0, -60.0, 90.0, 0.0), PhonePose(180.0, 89.0, 300.0, 0.0, 60.0, 90.0, 0.0),
                PhonePose(180.0, 89.0, 300.0, 0.0, 60.0, -90.0, 0.0), PhonePose(180.0, 89.0, 300.0, 0.0, -60.0, -90.0, 0.0)), 1300),
            Page(R.string.t9_title, R.string.t9_body, GuideView.Mode.SCAN, true, listOf(PhonePose(160.0, 35.0, 320.0))),
        )
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Ui.edgeToEdge(this)
        b = ActivityTutorialBinding.inflate(layoutInflater); setContentView(b.root)
        Ui.padSystemBars(b.content)
        page = intent.getIntExtra("page", 0).coerceIn(0, pages.size - 1)
        b.btnNext.setOnClickListener { if (page < pages.size - 1) { page++; show() } else finish() }
        b.btnPrev.setOnClickListener { if (page > 0) { page--; show() } }
        b.btnClose.setOnClickListener { finish() }
        show()
    }

    private fun show() {
        val p = pages[page]
        b.title.text = getString(p.title); b.body.text = getString(p.body)
        b.guide.mode = p.mode; b.guide.showObject = p.obj; b.guide.keyframes = p.frames; b.guide.segmentMs = p.segMs; b.guide.target = null; b.guide.live = null
        if (p.slots) {
            val m = HashMap<String, Int>(); for (bb in 0 until 3) for (a in 0 until 6) m["$bb-$a"] = 0
            b.guide.slots = m
        } else b.guide.slots = emptyMap()
        b.guide.invalidate()
        b.dots.text = (0 until pages.size).joinToString(" ") { if (it == page) "●" else "○" }
        b.btnPrev.visibility = if (page == 0) View.INVISIBLE else View.VISIBLE
        b.btnNext.text = getString(if (page == pages.size - 1) R.string.t_start else R.string.t_next)
        b.scroll.scrollTo(0, 0)
    }
}
