package ir.scan3d.app

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

/**
 * Guided camera calibration (what mrcal and the photogrammetry literature ask for):
 *   - the board photographed close, filling most of the frame
 *   - every region of the frame covered by board corners (distortion lives at the edges)
 *   - tilts of ~30-50 degrees from four sides, plus frontal views
 *   - the phone rolled 90 degrees for some shots (separates focal length from principal point)
 *   - two distances (near / far), focus locked at the scanning distance
 *   - no motion blur, no changing focus
 *
 * The planner turns each captured frame's board corners into coverage and tells the user
 * what is still missing. The actual solve runs on the phone (engine/Calibrator.kt) with
 * uncertainty estimates once the user presses "compute".
 */
class CalibPlanner(val width: Int, val height: Int) {
    private val gridX = 4; private val gridY = 6
    private val cells = IntArray(gridX * gridY)
    val views = ArrayList<View>()

    data class View(val tiltDeg: Double, val tiltDir: Int, val rolled: Boolean, val fill: Double, val corners: Int, val sharp: Double)

    /** tiltDir: 0 frontal, 1 top, 2 right, 3 bottom, 4 left (where the far edge of the board is). */
    fun add(board: BoardObs?, rolled: Boolean, sharp: Double): View? {
        board ?: return null
        val n = board.ids.size
        if (n < 12) return null
        var minx = 1e9; var maxx = -1e9; var miny = 1e9; var maxy = -1e9
        for (i in 0 until n) {
            val x = board.pts[2 * i].toDouble(); val y = board.pts[2 * i + 1].toDouble()
            minx = min(minx, x); maxx = max(maxx, x); miny = min(miny, y); maxy = max(maxy, y)
            val cx = (x / width * gridX).toInt().coerceIn(0, gridX - 1); val cy = (y / height * gridY).toInt().coerceIn(0, gridY - 1)
            cells[cy * gridX + cx]++
        }
        val fill = ((maxx - minx) * (maxy - miny)) / (width.toDouble() * height)
        // tilt from the foreshortening of the board: ratio of square sizes between far and near rows
        val (tilt, dir) = estimateTilt(board)
        val v = View(tilt, dir, rolled, fill, n, sharp)
        views.add(v)
        return v
    }

    private fun estimateTilt(b: BoardObs): Pair<Double, Int> {
        // compare mean spacing of corners along board columns at the two ends of the board
        val byRow = HashMap<Int, ArrayList<Pair<Double, Double>>>()
        for (i in b.ids.indices) {
            val id = b.ids[i]; val r = id / (Detector.SQUARES_X - 1)
            byRow.getOrPut(r) { ArrayList() }.add(b.pts[2 * i].toDouble() to b.pts[2 * i + 1].toDouble())
        }
        val rows = byRow.filter { it.value.size >= 3 }.toSortedMap()
        if (rows.size < 3) return 0.0 to 0
        fun spacing(pts: List<Pair<Double, Double>>): Double {
            val s = pts.sortedBy { it.first }
            var sum = 0.0; var k = 0
            for (i in 1 until s.size) { sum += hypot(s[i].first - s[i - 1].first, s[i].second - s[i - 1].second); k++ }
            return if (k > 0) sum / k else 0.0
        }
        val first = rows.values.first(); val last = rows.values.last()
        val s1 = spacing(first); val s2 = spacing(last)
        if (s1 <= 0 || s2 <= 0) return 0.0 to 0
        val ratio = min(s1, s2) / max(s1, s2)
        // for a board of length L at distance Z tilted by t: ratio ~ (Z - L/2 sin t)/(Z + L/2 sin t); invert roughly
        val tilt = Math.toDegrees(Math.asin(((1 - ratio) / (1 + ratio) * 2.0).coerceIn(0.0, 1.0)))
        val c1 = first.map { it.second }.average() to first.map { it.first }.average()
        val c2 = last.map { it.second }.average() to last.map { it.first }.average()
        val farIsFirst = s1 < s2
        val far = if (farIsFirst) c1 else c2; val near = if (farIsFirst) c2 else c1
        val dy = far.first - near.first; val dx = far.second - near.second
        val dir = if (tilt < 12) 0 else {
            val a = Math.toDegrees(atan2(dy, dx))
            when { abs(a) <= 45 -> 2; a > 45 && a < 135 -> 3; a <= -45 && a > -135 -> 1; else -> 4 }
        }
        return tilt to dir
    }

    data class Status(val cellsCovered: Int, val cellsTotal: Int, val frontal: Int, val tilted: IntArray, val rolled: Int, val close: Int, val total: Int, val next: String, val done: Boolean)

    fun status(): Status {
        val covered = cells.count { it >= 3 }
        val frontal = views.count { it.tiltDeg < 12 }
        val tilted = IntArray(5); views.filter { it.tiltDeg >= 20 }.forEach { tilted[it.tiltDir]++ }
        val rolled = views.count { it.rolled }
        val close = views.count { it.fill > 0.35 }
        val total = views.size
        val next = when {
            total < 3 -> "اول ۳ عکس از روبه‌رو بگیر؛ صفحه بیشتر کادر رو پر کنه."
            close < 6 -> "نزدیک‌تر شو تا صفحه حداقل یک‌سوم کادر رو پر کنه (${close}/6)."
            tilted[1] < 2 -> "گوشی رو حدود ۴۰ درجه کج کن، طوری که لبه‌ی بالایی صفحه دورتر باشه (${tilted[1]}/2)."
            tilted[3] < 2 -> "همین کار از پایین: لبه‌ی پایینی صفحه دورتر (${tilted[3]}/2)."
            tilted[2] < 2 -> "کج از راست: لبه‌ی راست دورتر (${tilted[2]}/2)."
            tilted[4] < 2 -> "کج از چپ: لبه‌ی چپ دورتر (${tilted[4]}/2)."
            rolled < 4 -> "گوشی رو افقی (۹۰ درجه چرخیده) بگیر و ۴ عکس بگیر (${rolled}/4)."
            covered < cellsTotal() - 2 -> "صفحه رو به گوشه‌ها و لبه‌های کادر ببر؛ ${cellsTotal() - covered} خونه هنوز خالیه."
            total < 25 -> "خوبه. چند عکس دیگه با فاصله و زاویه‌های متفاوت بگیر (${total}/25)."
            else -> "کالیبراسیون کامله. دکمه‌ی «محاسبه» رو بزن."
        }
        val done = total >= 25 && close >= 6 && rolled >= 4 && covered >= cellsTotal() - 2 && (1..4).all { tilted[it] >= 2 }
        return Status(covered, cellsTotal(), frontal, tilted, rolled, close, total, next, done)
    }

    fun cellsTotal() = gridX * gridY
    fun cellCounts(): IntArray = cells.copyOf()
}
