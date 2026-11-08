package ir.scan3d.app.engine

import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.ByteArrayOutputStream
import kotlin.math.abs
import kotlin.math.sqrt

/** Ground-truth tests for the pure-Kotlin engine, mirroring the Python reference tests. */
class EngineTest {
    private fun loadScan(): Triple<List<FrameObs>, Map<Int, Vec3>, JSONObject> {
        val txt = javaClass.classLoader!!.getResourceAsStream("synthetic_scan.json")!!.bufferedReader().readText()
        val j = JSONObject(txt)
        val shots = j.getJSONObject("scan").getJSONArray("shots")
        val frames = ArrayList<FrameObs>()
        for (i in 0 until shots.length()) {
            val s = shots.getJSONObject(i)
            val orient = s.getJSONObject("exif").getInt("orientation"); val wd = s.getInt("w"); val hd = s.getInt("h")
            val ch = s.getJSONObject("charuco"); val idsA = ch.getJSONArray("ids"); val ptsA = ch.getJSONArray("pts")
            val ids = IntArray(idsA.length()) { idsA.getInt(it) }
            val pts = DoubleArray(ptsA.length()) { ptsA.getDouble(it) }
            val sp = toSensor(pts, orient, wd, hd)
            val tg = s.getJSONObject("targets"); val stickers = HashMap<Int, DoubleArray>()
            for (k in tg.keys()) { val a = tg.getJSONArray(k); stickers[k.toInt()] = toSensor(DoubleArray(8) { a.getDouble(it) }, orient, wd, hd) }
            frames.add(FrameObs(i, ids, sp, stickers, true))
        }
        val truth = HashMap<Int, Vec3>()
        val t = j.getJSONObject("truth"); for (k in t.keys()) { val a = t.getJSONArray(k); truth[k.toInt()] = Vec3(a.getDouble(0), a.getDouble(1), a.getDouble(2)) }
        return Triple(frames, truth, j)
    }

    /** Same mapping as reconstruct.to_sensor: displayed (EXIF-applied) -> stored sensor frame. */
    private fun toSensor(p: DoubleArray, orient: Int, wd: Int, hd: Int): DoubleArray {
        val out = DoubleArray(p.size)
        var i = 0
        while (i < p.size) { val x = p[i]; val y = p[i + 1]
            when (orient) { 6 -> { out[i] = y; out[i + 1] = wd - 1 - x }; 8 -> { out[i] = hd - 1 - y; out[i + 1] = x }; 3 -> { out[i] = wd - 1 - x; out[i + 1] = hd - 1 - y }; else -> { out[i] = x; out[i + 1] = y } }
            i += 2 }
        return out
    }

    @Test
    fun solverRecoversCameraAndStickerCentres() {
        val (frames, truth, j) = loadScan()
        val solver = Solver(25.0); solver.log = { println(it) }
        val t0 = System.currentTimeMillis()
        val r = solver.solve(frames, 4000, 3000)
        println("solve took ${System.currentTimeMillis() - t0} ms; sigma0 ${r.sigma0}; f ${r.camera.f} cx ${r.camera.cx} cy ${r.camera.cy} k1 ${r.camera.k1} k2 ${r.camera.k2}")
        val cam = j.getJSONObject("camera")
        assertEquals(cam.getDouble("f"), r.camera.f, 2.0)
        assertEquals(cam.getDouble("cx"), r.camera.cx, 2.0)
        assertEquals(cam.getDouble("cy"), r.camera.cy, 2.0)
        assertEquals(cam.getDouble("k1"), r.camera.k1, 0.003)
        var worst = 0.0; var sum = 0.0
        for (s in r.stickers) { val e = (s.centre - truth[s.id]!!).norm(); worst = maxOf(worst, e); sum += e; println("sticker ${s.id}: err %.4f mm, sigma %.4f, edges %s".format(e, sqrt(s.sigma.sumOf { it * it }), s.edgesMm.joinToString { "%.2f".format(it) })) }
        println("mean centre error %.4f mm, worst %.4f; edge bias %.3f".format(sum / r.stickers.size, worst, r.edgeBiasMm))
        assertEquals(7, r.stickers.size)
        assertTrue("worst centre error $worst mm", worst < 0.05)
        assertTrue("edge bias should be about -0.22 mm, got ${r.edgeBiasMm}", r.edgeBiasMm < -0.15 && r.edgeBiasMm > -0.35)
    }

    @Test
    fun calibratorRecoversCamera() {
        val (frames, _, j) = loadScan()
        val cal = Calibrator(25.0); cal.log = { println(it) }
        val rolled = BooleanArray(frames.size) { it % 4 == 0 }   // 5 landscape frames in the synthetic set
        val r = cal.calibrate(frames, 4000, 3000, rolled)
        println("calib: f ${r.camera.f} +- ${r.sigma[0]}, c (${r.camera.cx}, ${r.camera.cy}), k1 ${r.camera.k1}, rms ${r.rms}, model ${r.model}, warnings ${r.warnings}")
        val cam = j.getJSONObject("camera")
        assertEquals(cam.getDouble("f"), r.camera.f, 3.0)
        assertEquals(cam.getDouble("k1"), r.camera.k1, 0.004)
        assertTrue(r.sigma[0] > 0 && r.sigma[0] < 5)
    }

    @Test
    fun carvingReproducesABox() {
        // 12 synthetic silhouettes of a 50x50x27 box on the board, from the solved camera poses of the scan
        val (frames, _, j) = loadScan()
        val r = Solver(25.0).solve(frames, 4000, 3000)
        val W = 500; val H = 375; val s = W / 4000.0
        val cam = Camera(r.camera.f * s, r.camera.cx * s, r.camera.cy * s, r.camera.k1, r.camera.k2)
        val cx = 87.5; val cy = 125.0; val sz = doubleArrayOf(50.0, 50.0, 27.0)
        val views = ArrayList<SilView>()
        for (p in r.poses.filterNotNull()) {
            val mask = BooleanArray(W * H)
            // rasterise the box by projecting a dense set of its surface points (cheap and good enough)
            for (ix in 0..50) for (iy in 0..50) for (iz in 0..27) {
                val onSurface = ix == 0 || ix == 50 || iy == 0 || iy == 50 || iz == 0 || iz == 27
                if (!onSurface) continue
                val wp = Vec3(cx - 25 + ix, cy - 25 + iy, -iz.toDouble())
                val pr = cam.project(p.toCamera(wp)) ?: continue
                val u = pr[0].toInt(); val v = pr[1].toInt(); if (u in 0 until W && v in 0 until H) mask[v * W + u] = true
            }
            val filled = Silhouette.fillHoles(Silhouette.close(mask, W, H, 2), W, H)
            views.add(SilView(W, H, filled, cam, p))
        }
        val planes = listOf(PlaneCut(Vec3(cx + 10, cy - 10, 27.0), Vec3(0.0, 0.0, 1.0), null), PlaneCut(Vec3(cx, cy + 25, 13.5), Vec3(0.0, 1.0, 0.0), null), PlaneCut(Vec3(cx + 25, cy, 13.5), Vec3(1.0, 0.0, 0.0), null))
        val t0 = System.currentTimeMillis()
        val coarse = Carve.carve(views, arrayOf(doubleArrayOf(0.0, 175.0), doubleArrayOf(0.0, 250.0), doubleArrayOf(0.0, 100.0)), 2.0, 1.0, planes, 1)
        val b = Carve.insideBounds(coarse, 4.0)!!
        val g = Carve.carve(views, b, 0.6, 1.0, planes, 1)
        val mesh = MarchingCubes.run(g)
        val bb = mesh.bounds()
        println("carve+mc took ${System.currentTimeMillis() - t0} ms; tris ${mesh.triangleCount}; size %.1f x %.1f x %.1f; volume %.1f cm3".format(bb[1][0] - bb[0][0], bb[1][1] - bb[0][1], bb[1][2] - bb[0][2], mesh.volume() / 1000))
        assertEquals(50.0, bb[1][0] - bb[0][0], 2.5); assertEquals(50.0, bb[1][1] - bb[0][1], 2.5); assertEquals(27.0, bb[1][2] - bb[0][2], 1.5)
        assertTrue(mesh.volume() > 0)
        val out = ByteArrayOutputStream(); mesh.writeStl(out)
        assertEquals(84 + 50 * mesh.triangleCount, out.size())

        // robust fusion: one deliberately wrong silhouette (empty mask) must not destroy the object when 2 votes are required
        val bad = SilView(W, H, BooleanArray(W * H), cam, views[0].pose)
        val views2 = views + bad
        val coarse1 = Carve.carve(views2, arrayOf(doubleArrayOf(0.0, 175.0), doubleArrayOf(0.0, 250.0), doubleArrayOf(0.0, 100.0)), 2.0, 1.0, planes, 1)
        assertTrue("one empty silhouette carves everything with tolerance 1", Carve.insideBounds(coarse1, 4.0) == null)
        val coarse2 = Carve.carve(views2, arrayOf(doubleArrayOf(0.0, 175.0), doubleArrayOf(0.0, 250.0), doubleArrayOf(0.0, 100.0)), 2.0, 1.0, planes, 2)
        val b2 = Carve.insideBounds(coarse2, 4.0)!!
        val g2 = Carve.carve(views2, b2, 0.6, 1.0, planes, 2)
        val mesh2 = MarchingCubes.run(g2); val bb2 = mesh2.bounds()
        println("tolerant carve: size %.1f x %.1f x %.1f; volume %.1f cm3".format(bb2[1][0] - bb2[0][0], bb2[1][1] - bb2[0][1], bb2[1][2] - bb2[0][2], mesh2.volume() / 1000))
        assertEquals(50.0, bb2[1][0] - bb2[0][0], 2.5); assertEquals(27.0, bb2[1][2] - bb2[0][2], 1.5)
        assertEquals(mesh.volume(), mesh2.volume(), 0.03 * mesh.volume())
        val fr = Carve.insideFractions(views2, b2, 4.0)
        assertEquals(0.0, fr.last(), 1e-9); assertTrue(fr[0] > 0.2)
    }
}
