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
        println("sheet bend on the flat synthetic board: %.3f mm peak-to-valley".format(r.warp.peakToValley()))
        assertTrue("a flat board must not be reported as bent: ${r.warp.peakToValley()} mm", r.warp.peakToValley() < 0.3)
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
        // one photo with only 8 corners is skipped; this used to throw ArrayIndexOutOfBounds (frame indices vs filtered list)
        val cut = frames.mapIndexed { i, f -> if (i == 2) FrameObs(f.index, f.boardIds.copyOf(8), f.boardPx.copyOf(16), f.stickers, true) else f }
        val r2 = cal.calibrate(cut, 4000, 3000, rolled)
        assertEquals(frames.size - 1, r2.frames)
        assertEquals(cam.getDouble("f"), r2.camera.f, 4.0)
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

    // ------------------------------------------------------------------ helpers for the synthetic scenes below
    /** Camera looking at the board centre from a clock direction / elevation, optionally rolled about its axis. */
    private fun lookAt(azDeg: Double, elDeg: Double, dist: Double, rollDeg: Double = 0.0): Pose6 {
        val az = Math.toRadians(azDeg); val el = Math.toRadians(elDeg)
        val aim = Vec3(87.5, 125.0, 0.0)
        val c = Vec3(aim.x + dist * Math.sin(az) * Math.cos(el), aim.y - dist * Math.cos(az) * Math.cos(el), -dist * Math.sin(el))
        val f = (aim - c).unit(); val up = Vec3(0.0, 0.0, -1.0)
        var r = f.cross(up).unit(); var d = f.cross(r)
        val ro = Math.toRadians(rollDeg); val cr = Math.cos(ro); val sr = Math.sin(ro)
        val r2 = r * cr + d * sr; val d2 = d * cr - r * sr; r = r2; d = d2
        val R = arrayOf(doubleArrayOf(r.x, r.y, r.z), doubleArrayOf(d.x, d.y, d.z), doubleArrayOf(f.x, f.y, f.z))
        val t = doubleArrayOf(-(R[0][0] * c.x + R[0][1] * c.y + R[0][2] * c.z), -(R[1][0] * c.x + R[1][1] * c.y + R[1][2] * c.z), -(R[2][0] * c.x + R[2][1] * c.y + R[2][2] * c.z))
        return Pose6(Rot.toVec(R), t)
    }

    /** Convex hull (Andrew monotone chain) of 2D points given as x, y pairs. */
    private fun convexHull(pts: List<DoubleArray>): List<DoubleArray> {
        val p = pts.sortedWith(compareBy({ it[0] }, { it[1] }))
        fun cross(o: DoubleArray, a: DoubleArray, b: DoubleArray) = (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
        val h = ArrayList<DoubleArray>()
        for (q in p) { while (h.size >= 2 && cross(h[h.size - 2], h[h.size - 1], q) <= 0) h.removeAt(h.size - 1); h.add(q) }
        val lower = h.size + 1
        for (i in p.size - 2 downTo 0) { val q = p[i]; while (h.size >= lower && cross(h[h.size - 2], h[h.size - 1], q) <= 0) h.removeAt(h.size - 1); h.add(q) }
        h.removeAt(h.size - 1)
        return h
    }

    /**
     * The clean model is fitted to contour rays, not to the visual hull: an ideal tilted rounded box seen from
     * only 8 directions must come back to a tenth of a millimetre (the hull of 9 real views read 1.2 mm too wide).
     */
    @Test
    fun rayFitRecoversRoundedBoxFromEightViews() {
        val truth = doubleArrayOf(86.5, 124.1, Math.toRadians(3.0), 66.0, 62.0, 28.0, 11.0, 6.0, Math.toRadians(3.0), Math.toRadians(-2.5), 4.0)
        val zs = 0.0
        val model = RayFit.Model(truth, zs)
        // points on the true surface: bisection along rays from the body centre
        val surf = ArrayList<Vec3>()
        for (ie in 1 until 90) for (ia in 0 until 180) {
            val el = Math.toRadians(-90.0 + 2.0 * ie); val az = Math.toRadians(2.0 * ia)
            val dx = Math.cos(el) * Math.cos(az); val dy = Math.cos(el) * Math.sin(az); val dz = Math.sin(el)
            var lo = 0.0; var hi = 120.0
            for (it in 0 until 40) { val mid = (lo + hi) / 2; if (model.sdf(model.mx + mid * dx, model.my + mid * dy, model.mz + mid * dz) <= 0) lo = mid else hi = mid }
            surf.add(Vec3(model.mx + lo * dx, model.my + lo * dy, model.mz + lo * dz))
        }
        // resting: the sheet plane z = 0 touches the tilted body and does not cut it
        var touch = Double.MAX_VALUE
        var gx = 40.0; while (gx <= 135.0) { var gy = 80.0; while (gy <= 170.0) { touch = minOf(touch, model.sdf(gx, gy, 0.0)); gy += 0.25 }; gx += 0.25 }
        assertEquals("the tilted body must rest on the sheet", 0.0, touch, 0.02)
        val cam = Camera(2885.0, 2040.0, 1530.0, 0.05, -0.10)
        val rnd = java.util.Random(5)
        val rays = RayFit.Rays()
        var view = 0
        for (el in listOf(27.0, 67.0)) for (az in listOf(20.0, 110.0, 200.0, 290.0)) {
            val pose = lookAt(az, el, 330.0)
            val px = surf.mapNotNull { cam.project(pose.toCamera(Vec3(it.x, it.y, -it.z))) }       // z-up -> board frame
            val hull = convexHull(px)
            val R = pose.R(); val c = pose.centre()
            for (i in hull.indices) {
                val a = hull[i]; val b = hull[(i + 1) % hull.size]
                val n = maxOf(1, (Math.hypot(b[0] - a[0], b[1] - a[1]) / 6.0).toInt())
                for (k in 0 until n) rays.addPixel(cam, R, c, a[0] + (b[0] - a[0]) * k / n + 0.3 * rnd.nextGaussian(), a[1] + (b[1] - a[1]) * k / n + 0.3 * rnd.nextGaussian(), true, view, zs)
            }
            view++
        }
        val start = doubleArrayOf(88.0, 123.0, Math.toRadians(8.0), 70.0, 65.0, 26.5, 5.0, 3.0, 0.0, 0.0, 1.0)
        val t0 = System.currentTimeMillis()
        val fit = RayFit.fit(start, rays, DoubleArray(0), zs)!!
        val p = RayFit.canonical(fit.p)
        println("ray fit took ${System.currentTimeMillis() - t0} ms on ${rays.n} rays: %.3f x %.3f x %.3f rc %.2f rz %.2f rb %.2f tilt %.2f deg; %.0f%% within 0.5 mm, median %.3f; sigma W %.3f H %.3f".format(
            p[RayFit.W], p[RayFit.L], p[RayFit.H], p[RayFit.RC], p[RayFit.RZ], p[RayFit.RB], fit.tiltDeg, 100 * fit.within, fit.medianAbs, fit.sigma!![RayFit.W], fit.sigma!![RayFit.H]))
        assertEquals(66.0, p[RayFit.W], 0.15); assertEquals(62.0, p[RayFit.L], 0.15); assertEquals(28.0, p[RayFit.H], 0.15)
        assertEquals(11.0, p[RayFit.RC], 0.5); assertEquals(6.0, p[RayFit.RZ], 0.6); assertEquals(4.0, p[RayFit.RB], 0.8)
        assertEquals(model.tiltDeg, fit.tiltDeg, 0.3)
        assertEquals(86.5, p[RayFit.CX], 0.15); assertEquals(124.1, p[RayFit.CY], 0.15)
        assertTrue("fit score ${fit.within}", fit.within > 0.95)
        assertTrue("repeatability must be small on clean data: ${fit.sigma!![RayFit.W]}", fit.sigma!![RayFit.W] < 0.3 && fit.sigma!![RayFit.H] < 0.3)
        // the clean mesh: untilted, flat on z = 0, every vertex on the model surface
        val clean = RayFit.mesh(p); val b = clean.bounds()
        assertEquals(0.0, b[0][2], 1e-4); assertEquals(p[RayFit.H], b[1][2], 1e-3)
        val flat = RayFit.Model(p.copyOf().also { it[RayFit.TX] = 0.0; it[RayFit.TY] = 0.0 }, 0.0)
        var worst = 0.0
        for (i in 0 until clean.vertexCount) worst = maxOf(worst, abs(flat.sdf(clean.verts[3 * i].toDouble(), clean.verts[3 * i + 1].toDouble(), clean.verts[3 * i + 2].toDouble())))
        println("clean mesh: ${clean.triangleCount} triangles, volume %.2f cm3, worst vertex %.5f mm off the surface".format(clean.volume() / 1000, worst))
        assertTrue("vertex off the surface by $worst mm", worst < 2e-3)
        // volume of a rounded box is a bit less than W L H
        assertTrue(clean.volume() > 0.80 * p[RayFit.W] * p[RayFit.L] * p[RayFit.H] && clean.volume() < p[RayFit.W] * p[RayFit.L] * p[RayFit.H])
    }

    /** A sticker centre on the surface pins the height even when no view sees the top edge well. */
    @Test
    fun rayFitUsesStickerCentres() {
        val truth = doubleArrayOf(90.0, 120.0, 0.0, 60.0, 50.0, 30.0, 8.0, 4.0, 0.0, 0.0, 3.0)
        val model = RayFit.Model(truth, 0.0)
        val cam = Camera(2885.0, 2040.0, 1530.0, 0.0, 0.0)
        val surf = ArrayList<Vec3>()
        for (ie in 1 until 60) for (ia in 0 until 120) {
            val el = Math.toRadians(-90.0 + 3.0 * ie); val az = Math.toRadians(3.0 * ia)
            val dx = Math.cos(el) * Math.cos(az); val dy = Math.cos(el) * Math.sin(az); val dz = Math.sin(el)
            var lo = 0.0; var hi = 120.0
            for (it in 0 until 40) { val mid = (lo + hi) / 2; if (model.sdf(model.mx + mid * dx, model.my + mid * dy, model.mz + mid * dz) <= 0) lo = mid else hi = mid }
            surf.add(Vec3(model.mx + lo * dx, model.my + lo * dy, model.mz + lo * dz))
        }
        val rays = RayFit.Rays(); var view = 0
        for (az in listOf(0.0, 72.0, 144.0, 216.0, 288.0)) {            // top-down views only: the height is barely visible in the outlines
            val pose = lookAt(az, 82.0, 330.0); val R = pose.R(); val c = pose.centre()
            val hull = convexHull(surf.mapNotNull { cam.project(pose.toCamera(Vec3(it.x, it.y, -it.z))) })
            for (i in hull.indices) { val a = hull[i]; val b = hull[(i + 1) % hull.size]; val n = maxOf(1, (Math.hypot(b[0] - a[0], b[1] - a[1]) / 8.0).toInt())
                for (k in 0 until n) rays.addPixel(cam, R, c, a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, true, view, 0.0) }
            view++
        }
        // three stickers on the top face: their printed faces are 0.1 mm above the surface
        val st = doubleArrayOf(80.0, 112.0, 30.1, 100.0, 118.0, 30.1, 92.0, 133.0, 30.1)
        val start = doubleArrayOf(91.0, 121.0, 0.05, 63.0, 52.0, 26.0, 6.0, 3.0, 0.0, 0.0, 2.0)
        val fit = RayFit.fit(start, rays, st, 0.0, jackknife = false)!!
        val p = RayFit.canonical(fit.p)
        println("ray fit with stickers: H %.3f (truth 30), sticker residuals %s".format(p[RayFit.H], fit.stickerResiduals.joinToString { "%.3f".format(it) }))
        assertEquals(30.0, p[RayFit.H], 0.1)
        assertEquals(60.0, p[RayFit.W], 0.2); assertEquals(50.0, p[RayFit.L], 0.2)
    }

    /** Paper on a carpet is not flat: the solver must measure the bend instead of pushing it into the camera. */
    @Test
    fun solverMeasuresTheSheetBend() {
        val cam = Camera(2900.0, 2040.0, 1530.0, 0.10, -0.20)
        val warp = SheetWarp.flat(175.0, 250.0); warp.k[0] = 3.0; warp.k[1] = -1.0; warp.k[2] = 1.5
        val rnd = java.util.Random(9)
        val frames = ArrayList<FrameObs>()
        val poses = listOf(Triple(10.0, 42.0, 0.0), Triple(50.0, 68.0, 0.0), Triple(95.0, 44.0, 0.0), Triple(140.0, 72.0, 90.0), Triple(185.0, 41.0, 0.0), Triple(225.0, 66.0, 0.0),
            Triple(265.0, 43.0, 0.0), Triple(300.0, 74.0, 90.0), Triple(335.0, 52.0, 0.0), Triple(160.0, 58.0, 0.0))
        for ((i, pz) in poses.withIndex()) {
            val pose = lookAt(pz.first, pz.second, 360.0, pz.third)
            val ids = ArrayList<Int>(); val px = ArrayList<Double>()
            for (id in 0 until 54) {
                val P = warp.lift(Vec3(((id % 6) + 1) * 25.0, ((id / 6) + 1) * 25.0, 0.0))
                val pr = cam.project(pose.toCamera(P)) ?: continue
                if (pr[0] < 5 || pr[1] < 5 || pr[0] > 4075 || pr[1] > 3055) continue
                ids.add(id); px.add(pr[0] + 0.3 * rnd.nextGaussian()); px.add(pr[1] + 0.3 * rnd.nextGaussian())
            }
            frames.add(FrameObs(i, ids.toIntArray(), px.toDoubleArray(), emptyMap(), true))
        }
        val solver = Solver(25.0); solver.log = { println(it) }
        val r = solver.solve(frames, 4080, 3060)
        println("bend: truth 3.00 -1.00 1.50 -> %.2f %.2f %.2f (sigma %s); sigma0 %.3f; f %.1f k1 %.4f k2 %.4f".format(r.warp.k[0], r.warp.k[1], r.warp.k[2],
            r.warpSigma?.joinToString { "%.2f".format(it) }, r.sigma0, r.camera.f, r.camera.k1, r.camera.k2))
        assertEquals(3.0, r.warp.k[0], 0.45); assertEquals(-1.0, r.warp.k[1], 0.45); assertEquals(1.5, r.warp.k[2], 0.45)
        assertEquals(warp.peakToValley(), r.warp.peakToValley(), 0.5)
        assertEquals(0.3, r.sigma0, 0.08)
        assertEquals(2900.0, r.camera.f, 20.0)
        // the same data forced flat fits visibly worse: that difference is what the bend explains
        val flatSolver = Solver(25.0); flatSolver.estimateWarp = false
        val rf = flatSolver.solve(frames, 4080, 3060)
        println("forced flat: sigma0 %.3f, f %.1f".format(rf.sigma0, rf.camera.f))
        assertTrue("ignoring a 3 mm bend must cost residual: ${rf.sigma0} vs ${r.sigma0}", rf.sigma0 > 1.5 * r.sigma0)
    }

    /** Sub-pixel edge of a bright disc floating 20 mm above the board: where the board does not explain the step. */
    @Test
    fun contourLocatesObjectEdgeBelowAPixel() {
        val tpl = fakeTemplate()
        val cam = Camera(1000.0, 500.0, 375.0, 0.02, 0.0)
        val W = 1000; val H = 750
        val pose = lookAt(200.0, 60.0, 350.0)
        val Rm = pose.R(); val C = pose.centre(); val rnd = java.util.Random(4)
        val discC = doubleArrayOf(90.0, 122.0); val discR = 26.0; val discZ = -20.0          // board frame: z into the board
        val photo = ByteArray(W * H); val truth = BooleanArray(W * H)
        for (v in 0 until H) for (u in 0 until W) {
            var acc = 0.0; var hits = 0
            for (sv in 0 until 3) for (su in 0 until 3) {                                  // 3 x 3 supersampling: honest partial pixels
                val n = cam.undistort(u + (su - 1) / 3.0, v + (sv - 1) / 3.0)
                val dx = Rm[0][0] * n[0] + Rm[1][0] * n[1] + Rm[2][0]; val dy = Rm[0][1] * n[0] + Rm[1][1] * n[1] + Rm[2][1]; val dz = Rm[0][2] * n[0] + Rm[1][2] * n[1] + Rm[2][2]
                var g = 120.0
                val sd = (discZ - C.z) / dz; val X1 = C.x + sd * dx; val Y1 = C.y + sd * dy
                if (Math.hypot(X1 - discC[0], Y1 - discC[1]) <= discR) { g = 232.0; hits++ }
                else { val s0 = -C.z / dz; val X = C.x + s0 * dx; val Y = C.y + s0 * dy
                    val t = tpl.sample(X, Y); if (t >= 0f) g = 45.0 + t * 0.62 * (0.8 + 0.3 * X / 175.0) }
                acc += g
            }
            truth[v * W + u] = hits >= 5
            photo[v * W + u] = (acc / 9 + rnd.nextGaussian() * 2.0).toInt().coerceIn(0, 255).toByte()
        }
        val res = Silhouette.extract(photo, W, H, cam, pose, tpl)!!
        // a mask as an AI segmenter would give it: right shape, a couple of pixels generous
        val mask = Silhouette.dilate(truth, W, H, 2)
        val sil = SilView(W, H, mask, cam, pose)
        val pts = Contour.extract(mask, sil.sdf, res.valid, W, H, res.levels, photo, W, H, cam, pose, null, tpl)
        val err = ArrayList<Double>()
        for (p in pts) if (p.edge) {
            val n = cam.undistort(p.u, p.v)
            val dx = Rm[0][0] * n[0] + Rm[1][0] * n[1] + Rm[2][0]; val dy = Rm[0][1] * n[0] + Rm[1][1] * n[1] + Rm[2][1]; val dz = Rm[0][2] * n[0] + Rm[1][2] * n[1] + Rm[2][2]
            val sd = (discZ - C.z) / dz
            err.add(abs(Math.hypot(C.x + sd * dx - discC[0], C.y + sd * dy - discC[1]) - discR))
        }
        err.sort()
        val mmPerPx = 350.0 / 1000.0
        println("contour: ${pts.size} boundary points, ${err.size} measured edges; error median %.3f px, 90%% %.3f px".format(err[err.size / 2] / mmPerPx, err[(err.size * 9) / 10] / mmPerPx))
        assertTrue("too few measured edges: ${err.size} of ${pts.size}", err.size > 0.25 * pts.size && err.size > 40)
        assertTrue("median edge error ${err[err.size / 2] / mmPerPx} px", err[err.size / 2] / mmPerPx < 0.35)
        assertTrue("90th percentile ${err[(err.size * 9) / 10] / mmPerPx} px", err[(err.size * 9) / 10] / mmPerPx < 1.2)
    }

    /** Meshing: shared vertices, loose crumbs dropped, base skirt removed. */
    @Test
    fun meshKeepsTheObjectAndDropsCrumbsAndSkirt() {
        val vox = 0.5
        val g = Carve.Grid(0.0, 0.0, vox / 2, 120, 100, 60, vox)
        fun box(x: Double, y: Double, z: Double, x0: Double, x1: Double, y0: Double, y1: Double, z1: Double) = minOf(minOf(x - x0, x1 - x), minOf(y - y0, y1 - y), z1 - z)
        for (i in 0 until g.nx) for (j in 0 until g.ny) for (k in 0 until g.nz) {
            val x = g.x0 + i * vox; val y = g.y0 + j * vox; val z = g.z0 + k * vox
            val main = box(x, y, z, 10.0, 40.0, 10.0, 40.0, 20.0)               // 30 x 30 x 20 mm object
            val plate = box(x, y, z, 4.0, 46.0, 4.0, 46.0, 1.6)                 // thin shadow skirt around its base
            val crumb = minOf(box(x, y, z, 50.0, 53.0, 20.0, 23.0, 12.0), z - 9.0)   // a floating 3 mm crumb
            g.f[g.idx(i, j, k)] = maxOf(main, maxOf(plate, crumb)).toFloat()
        }
        val cleared = Carve.removeSkirt(g, 4.5)
        assertTrue("skirt columns must be cleared, got $cleared", cleared > 1000)
        val b = Carve.insideBounds(g, 0.0)!!
        assertEquals(10.0, b[1][0], 0.6); assertEquals(40.0, b[1][1], 0.6)      // y extent: the 4 mm wide skirt is gone
        val raw = MarchingCubes.run(g)
        assertTrue("marching cubes must share vertices: ${raw.vertexCount} vertices for ${raw.triangleCount} triangles", raw.vertexCount < raw.triangleCount)
        val one = raw.welded().largestComponent()
        assertTrue(one.triangleCount < raw.triangleCount)
        println("mesh: ${raw.triangleCount} -> ${one.triangleCount} triangles after dropping the crumb; volume %.2f cm3 (box 18.0)".format(one.volume() / 1000))
        assertEquals(18000.0, one.volume(), 600.0)
        val sm = one.taubin(8)
        assertEquals(one.volume(), sm.volume(), 0.02 * one.volume())
    }

    /** STL for the viewer: real edges keep their face normals, curved surfaces get smooth ones. */
    @Test
    fun stlLoaderKeepsEdgesSharpAndCurvesSmooth() {
        fun roundTrip(m: Mesh): Stl.Model {
            val f = java.io.File.createTempFile("scan3d", ".stl")
            try { m.writeStl(f); return Stl.load(f)!! } finally { f.delete() }
        }
        // a 20 mm cube: every corner must carry the normal of its own face (90 degree creases)
        val cv = floatArrayOf(0f, 0f, 0f, 20f, 0f, 0f, 20f, 20f, 0f, 0f, 20f, 0f, 0f, 0f, 20f, 20f, 0f, 20f, 20f, 20f, 20f, 0f, 20f, 20f)
        val ct = intArrayOf(0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7)
        val cube = roundTrip(Mesh(cv, ct))
        assertEquals(12, cube.triangles)
        assertEquals(0f, cube.min[0], 1e-6f); assertEquals(20f, cube.max[2], 1e-6f)
        val cd = FloatArray(cube.vertexCount * 6); cube.data.get(cd); cube.data.position(0)
        for (c in 0 until cube.vertexCount) {
            val nx = abs(cd[6 * c + 3]); val ny = abs(cd[6 * c + 4]); val nz = abs(cd[6 * c + 5])
            assertEquals("cube corner $c must have an axis-aligned normal ($nx, $ny, $nz)", 1f, maxOf(nx, ny, nz), 1e-5f)
            assertEquals(1f, nx + ny + nz, 1e-5f)
        }
        // a sphere of radius 10: every corner normal must be radial (smooth), far closer than the 15 degree facets
        val nA = 24; val nE = 11
        val sv = ArrayList<Float>(); val st = ArrayList<Int>()
        for (r in 0 until nE) { val el = Math.toRadians(-90.0 + 180.0 * (r + 1) / (nE + 1))
            for (a in 0 until nA) { val az = 2 * Math.PI * a / nA; sv.add((10 * Math.cos(el) * Math.cos(az)).toFloat()); sv.add((10 * Math.cos(el) * Math.sin(az)).toFloat()); sv.add((10 * Math.sin(el)).toFloat()) } }
        val top = sv.size / 3; sv.add(0f); sv.add(0f); sv.add(10f)
        val bot = sv.size / 3; sv.add(0f); sv.add(0f); sv.add(-10f)
        for (r in 0 until nE - 1) for (a in 0 until nA) { val b = (a + 1) % nA
            st.add(r * nA + a); st.add(r * nA + b); st.add((r + 1) * nA + a); st.add(r * nA + b); st.add((r + 1) * nA + b); st.add((r + 1) * nA + a) }
        for (a in 0 until nA) { val b = (a + 1) % nA; st.add((nE - 1) * nA + a); st.add((nE - 1) * nA + b); st.add(top); st.add(b); st.add(a); st.add(bot) }
        val sphere = roundTrip(Mesh(sv.toFloatArray(), st.toIntArray()))
        val sd = FloatArray(sphere.vertexCount * 6); sphere.data.get(sd)
        var worst = 1.0
        for (c in 0 until sphere.vertexCount) {
            val x = sd[6 * c]; val y = sd[6 * c + 1]; val z = sd[6 * c + 2]
            val dot = (x * sd[6 * c + 3] + y * sd[6 * c + 4] + z * sd[6 * c + 5]) / 10.0
            if (dot < worst) worst = dot
        }
        println("stl: sphere corner normals within %.2f deg of radial".format(Math.toDegrees(Math.acos(worst.coerceIn(-1.0, 1.0)))))
        assertTrue("sphere normals must be smooth (worst cos $worst)", worst > Math.cos(Math.toRadians(3.0)))
    }
}
