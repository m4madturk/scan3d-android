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

    /** A synthetic ChArUco-like template: 7x10 squares of 25 mm at 4 px/mm, random 6x6 "bits" in every black square. */
    private fun fakeTemplate(): Silhouette.BoardTemplate {
        val ppm = 4.0; val w = 700; val h = 1000; val img = ByteArray(w * h)
        val rnd = java.util.Random(3)
        val bits = Array(70) { IntArray(36) { if (rnd.nextBoolean()) 1 else 0 } }
        for (y in 0 until h) for (x in 0 until w) {
            val sx = x / 100; val sy = y / 100
            val black = (sx + sy) % 2 == 0
            var v = if (black) 0 else 255
            if (black) { val lx = x - sx * 100; val ly = y - sy * 100
                if (lx in 14..85 && ly in 14..85) { val bx = (lx - 14) / 12; val by = (ly - 14) / 12; if (bits[sy * 7 + sx][by * 6 + bx] == 1) v = 255 } }
            img[y * w + x] = v.toByte()
        }
        return Silhouette.BoardTemplate(img, w, h, ppm)
    }

    @Test
    fun silhouetteSeparatesWhiteObjectFromWhiteSquares() {
        val tpl = fakeTemplate()
        val cam = Camera(1000.0, 500.0, 375.0, 0.02, 0.0)
        val W = 1000; val H = 750
        // camera 350 mm from the board centre, 55 degrees elevation, looking at the centre
        val az = Math.toRadians(200.0); val el = Math.toRadians(55.0); val dist = 350.0
        val aim = Vec3(87.5, 125.0, 0.0)
        val c = Vec3(aim.x + dist * Math.sin(az) * Math.cos(el), aim.y - dist * Math.cos(az) * Math.cos(el), -dist * Math.sin(el))
        val f = (aim - c).unit(); val up = Vec3(0.0, 0.0, -1.0)
        val r = f.cross(up).unit(); val d = f.cross(r)
        val R = arrayOf(doubleArrayOf(r.x, r.y, r.z), doubleArrayOf(d.x, d.y, d.z), doubleArrayOf(f.x, f.y, f.z))
        val t = Vec3(-(R[0][0] * c.x + R[0][1] * c.y + R[0][2] * c.z), -(R[1][0] * c.x + R[1][1] * c.y + R[1][2] * c.z), -(R[2][0] * c.x + R[2][1] * c.y + R[2][2] * c.z))
        val pose = Pose6(Rot.toVec(R), t.toArray())
        // render the "photo": board through the camera (same ray-plane model), lighting gradient, noise,
        // a white box object 50x40x20 mm at the board centre with a soft shadow, and carpet outside the board
        val photo = ByteArray(W * H); val truthObj = BooleanArray(W * H)
        val Rm = pose.R(); val C = pose.centre(); val rnd = java.util.Random(11)
        val box = doubleArrayOf(62.5, 112.5, 105.0, 137.5)   // x0 y0 x1 y1 on the board (base of the box)
        for (v in 0 until H) for (u in 0 until W) {
            val n = cam.undistort(u.toDouble(), v.toDouble())
            val dx = Rm[0][0] * n[0] + Rm[1][0] * n[1] + Rm[2][0]; val dy = Rm[0][1] * n[0] + Rm[1][1] * n[1] + Rm[2][1]; val dz = Rm[0][2] * n[0] + Rm[1][2] * n[1] + Rm[2][2]
            var g = 120 + (u % 37) / 6                                     // carpet-ish texture
            if (dz != 0.0) {
                // top face of the box at z = -20 (above the board), then the board plane
                val s20 = (-20.0 - C.z) / dz
                if (s20 > 0) { val X = C.x + s20 * dx; val Y = C.y + s20 * dy
                    if (X in box[0]..box[2] && Y in box[1]..box[3]) { g = 228 + ((X - box[0]) / 42.5 * 12).toInt(); truthObj[v * W + u] = true } }
                if (!truthObj[v * W + u]) {
                    val s0 = -C.z / dz
                    if (s0 > 0) { val X = C.x + s0 * dx; val Y = C.y + s0 * dy
                        // box side faces: the ray hits a vertical face when the ground point is inside the footprint extended... keep it simple: footprint = object
                        if (X in box[0]..box[2] && Y in box[1]..box[3]) { g = 205; truthObj[v * W + u] = true }
                        else { val tx = (X * tpl.pxPerMm).toInt(); val ty = (Y * tpl.pxPerMm).toInt()
                            if (tx in 0 until tpl.w && ty in 0 until tpl.h) {
                                val printed = tpl.img[ty * tpl.w + tx].toInt() and 255
                                val light = 0.7 + 0.5 * (X / 175.0)                       // one side of the room brighter
                                var shade = 1.0
                                if (X in box[2]..(box[2] + 15) && Y in (box[1] - 5)..(box[3] + 5)) shade = 0.55   // soft shadow next to the box
                                g = (40 + printed * 0.75 * light * shade).toInt().coerceIn(0, 255)
                            } } } }
            }
            photo[v * W + u] = (g + rnd.nextGaussian() * 3).toInt().coerceIn(0, 255).toByte()
        }
        val res = Silhouette.extract(photo, W, H, cam, pose, tpl)!!
        var objInside = 0; var objN = 0; var boardInside = 0; var boardN = 0
        for (i in 0 until W * H) {
            // only judge pixels on the board (valid) : object truth vs board truth
            val n = cam.undistort((i % W).toDouble(), (i / W).toDouble())
            val dx = Rm[0][0] * n[0] + Rm[1][0] * n[1] + Rm[2][0]; val dy = Rm[0][1] * n[0] + Rm[1][1] * n[1] + Rm[2][1]; val dz = Rm[0][2] * n[0] + Rm[1][2] * n[1] + Rm[2][2]
            val s0 = -C.z / dz; if (s0 <= 0) continue
            val X = C.x + s0 * dx; val Y = C.y + s0 * dy
            if (X < 5 || Y < 5 || X > 170 || Y > 245) continue
            if (truthObj[i]) { objN++; if (res.mask[i]) objInside++ } else { boardN++; if (res.mask[i]) boardInside++ }
        }
        println("silhouette: board verified %.1f%%; object kept %.1f%%; board wrongly inside %.1f%%".format(100 * res.solidity, 100.0 * objInside / objN, 100.0 * boardInside / boardN))
        assertTrue("object must stay inside: %.1f%%".format(100.0 * objInside / objN), objInside > 0.985 * objN)
        assertTrue("enough of the board must be verified (carvable): inside %.1f%%".format(100.0 * boardInside / boardN), boardInside < 0.5 * boardN)
        assertTrue(res.solidity > 0.45)
    }

    @Test
    fun planarPnpSurvivesNoiseAndRoughCamera() {
        // real photos: ~1.5 px corner noise, and a camera guess that is 15 % off; every frame must still get a pose
        val (frames, _, j) = loadScan()
        val cam = j.getJSONObject("camera")
        val rough = Camera(cam.getDouble("f") * 1.15, 2000.0, 1500.0, 0.0, 0.0)
        val rnd = java.util.Random(7)
        var ok = 0
        for (fr in frames) {
            val obj = fr.boardIds.map { Vec3(((it % 6) + 1) * 25.0, ((it / 6) + 1) * 25.0, 0.0) }
            val img = (0 until fr.boardIds.size).map { doubleArrayOf(fr.boardPx[2 * it] + 1.5 * rnd.nextGaussian(), fr.boardPx[2 * it + 1] + 1.5 * rnd.nextGaussian()) }
            val p = Pnp.solve(obj, img, rough)
            if (p != null && Pnp.rms(obj, img, rough, p) < 25.0) ok++
        }
        println("pnp: $ok / ${frames.size} frames posed with noise + rough camera")
        assertEquals(frames.size, ok)
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

        // smoothing must not shrink or grow the model
        val sm = mesh.taubin(12); val sb = sm.bounds()
        println("taubin: volume %.1f -> %.1f cm3, size %.1f x %.1f x %.1f".format(mesh.volume() / 1000, sm.volume() / 1000, sb[1][0] - sb[0][0], sb[1][1] - sb[0][1], sb[1][2] - sb[0][2]))
        assertEquals(mesh.volume(), sm.volume(), 0.01 * mesh.volume())
        assertEquals(bb[1][2] - bb[0][2], sb[1][2] - sb[0][2], 0.3)

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
