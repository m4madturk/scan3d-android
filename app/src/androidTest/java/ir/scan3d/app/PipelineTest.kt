package ir.scan3d.app

import android.graphics.Bitmap
import android.util.Log
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import ir.scan3d.app.engine.Camera
import ir.scan3d.app.engine.Parallel
import ir.scan3d.app.engine.Pose6
import ir.scan3d.app.engine.RayFit
import ir.scan3d.app.engine.Rot
import ir.scan3d.app.engine.Silhouette
import ir.scan3d.app.engine.Vec3
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.opencv.android.OpenCVLoader
import java.io.File
import java.util.Random
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

/**
 * End to end with ground truth: photographs of a known rounded box on the printed sheet are ray-traced (lens
 * distortion, anti-aliasing, sensor noise, JPEG), the marker detections get pixel noise, and the whole pipeline
 * (bundle adjustment, silhouettes, carving, contour edges, model fit) has to give the box back.
 * Runs on the emulator in CI; the numbers are in the log under "e2e:".
 */
@RunWith(AndroidJUnit4::class)
class PipelineTest {
    private val iw = 1020; private val ih = 765                    // a quarter of the phone's 4080 x 3060 frame
    private val cam = Camera(721.0, 513.4, 379.2, 0.06, -0.12)     // focal length of that phone at this size, mild distortion
    // truth, z-up frame (mm): centre, yaw, W, L, H, corner radius, top-edge radius, no tilt, bottom-edge radius
    private val truth = doubleArrayOf(90.0, 122.0, Math.toRadians(20.0), 60.0, 44.0, 26.0, 9.0, 4.0, 0.0, 0.0, 2.0)
    private val model = RayFit.Model(truth, 0.0)

    /** A sticker on the box: centre, two in-plane axes and the outward normal (z-up frame). */
    private class Stk(val id: Int, val c: Vec3, val u: Vec3, val v: Vec3, val n: Vec3)

    private fun local(lx: Double, ly: Double, lz: Double): Vec3 {
        val c = Math.cos(truth[RayFit.TH]); val s = Math.sin(truth[RayFit.TH])
        return Vec3(truth[RayFit.CX] + c * lx - s * ly, truth[RayFit.CY] + s * lx + c * ly, lz)
    }
    private fun dir(lx: Double, ly: Double, lz: Double): Vec3 { val o = local(0.0, 0.0, 0.0); val p = local(lx, ly, lz); return p - o }

    private val stickers: List<Stk> by lazy {
        val w = truth[RayFit.W]; val l = truth[RayFit.L]; val h = truth[RayFit.H]
        listOf(
            Stk(80, local(-13.0, 4.0, h), dir(1.0, 0.0, 0.0), dir(0.0, 1.0, 0.0), dir(0.0, 0.0, 1.0)),
            Stk(81, local(12.0, -7.0, h), dir(1.0, 0.0, 0.0), dir(0.0, 1.0, 0.0), dir(0.0, 0.0, 1.0)),
            Stk(82, local(-8.0, l / 2, 12.0), dir(1.0, 0.0, 0.0), dir(0.0, 0.0, 1.0), dir(0.0, 1.0, 0.0)),
            Stk(83, local(9.0, -l / 2, 12.0), dir(1.0, 0.0, 0.0), dir(0.0, 0.0, 1.0), dir(0.0, -1.0, 0.0)),
            Stk(84, local(w / 2, 3.0, 12.0), dir(0.0, 1.0, 0.0), dir(0.0, 0.0, 1.0), dir(1.0, 0.0, 0.0)),
            Stk(85, local(-w / 2, -2.0, 12.0), dir(0.0, 1.0, 0.0), dir(0.0, 0.0, 1.0), dir(-1.0, 0.0, 0.0)))
    }

    /** Camera at azimuth / elevation / distance around the middle of the sheet (board frame: z into the sheet). */
    private fun lookAt(azDeg: Double, elDeg: Double, dist: Double, rollDeg: Double): Pose6 {
        val az = Math.toRadians(azDeg); val el = Math.toRadians(elDeg)
        val aim = Vec3(87.5, 125.0, -8.0)
        val c = Vec3(aim.x + dist * Math.sin(az) * Math.cos(el), aim.y - dist * Math.cos(az) * Math.cos(el), aim.z - dist * Math.sin(el))
        val f = (aim - c).unit(); val up = Vec3(0.0, 0.0, -1.0)
        var r = f.cross(up).unit(); var d = f.cross(r)
        val ro = Math.toRadians(rollDeg); val cr = Math.cos(ro); val sr = Math.sin(ro)
        val r2 = r * cr + d * sr; val d2 = d * cr - r * sr; r = r2; d = d2
        val rot = arrayOf(doubleArrayOf(r.x, r.y, r.z), doubleArrayOf(d.x, d.y, d.z), doubleArrayOf(f.x, f.y, f.z))
        val t = doubleArrayOf(-(rot[0][0] * c.x + rot[0][1] * c.y + rot[0][2] * c.z), -(rot[1][0] * c.x + rot[1][1] * c.y + rot[1][2] * c.z), -(rot[2][0] * c.x + rot[2][1] * c.y + rot[2][2] * c.z))
        return Pose6(Rot.toVec(rot), t)
    }

    /** Distance along a ray (z-up frame, unit direction) to the box, or -1. */
    private fun hit(ox: Double, oy: Double, oz: Double, ux: Double, uy: Double, uz: Double): Double {
        val tc = (model.mx - ox) * ux + (model.my - oy) * uy + (model.mz - oz) * uz
        val qx = ox + tc * ux - model.mx; val qy = oy + tc * uy - model.my; val qz = oz + tc * uz - model.mz
        val rad = model.radius; val d2 = rad * rad - (qx * qx + qy * qy + qz * qz)
        if (d2 <= 0) return -1.0
        val half = sqrt(d2); var t = max(0.0, tc - half); val end = tc + half
        var k = 0
        while (t < end && k++ < 96) {
            val s = model.sdf(ox + t * ux, oy + t * uy, oz + t * uz)
            if (s < 2e-3) return t
            t += s
        }
        return -1.0
    }

    /** Gray value seen along a ray. */
    private fun shade(tpl: Silhouette.BoardTemplate, ox: Double, oy: Double, oz: Double, ux: Double, uy: Double, uz: Double): Double {
        val t = hit(ox, oy, oz, ux, uy, uz)
        if (t >= 0) {
            val x = ox + t * ux; val y = oy + t * uy; val z = oz + t * uz
            for (s in stickers) {
                val rx = x - s.c.x; val ry = y - s.c.y; val rz = z - s.c.z
                if (abs(rx * s.n.x + ry * s.n.y + rz * s.n.z) > 0.6) continue
                val a = abs(rx * s.u.x + ry * s.u.y + rz * s.u.z); val b = abs(rx * s.v.x + ry * s.v.y + rz * s.v.z)
                if (a < 6.0 && b < 6.0) return if (a < 4.5 && b < 4.5) 38.0 else 226.0           // black marker field, white rim
            }
            val e = 0.01
            var nx = model.sdf(x + e, y, z) - model.sdf(x - e, y, z); var ny = model.sdf(x, y + e, z) - model.sdf(x, y - e, z); var nz = model.sdf(x, y, z + e) - model.sdf(x, y, z - e)
            val nn = sqrt(nx * nx + ny * ny + nz * nz); nx /= nn; ny /= nn; nz /= nn
            return 150.0 + 72.0 * max(0.0, nx * 0.30 + ny * (-0.36) + nz * 0.88)                  // a light plastic box under a ceiling light
        }
        if (uz > -1e-9) return 105.0
        val ts = -oz / uz; val x = ox + ts * ux; val y = oy + ts * uy
        if (x < -17.5 || x > 192.5 || y < -23.5 || y > 273.5) return 105.0                          // the table around the A4 sheet
        val s = tpl.sample(x, y)
        return if (s < 0f) 214.0 else 40.0 + 174.0 * s / 255.0
    }

    private fun render(pose: Pose6, tpl: Silhouette.BoardTemplate, seed: Int): ByteArray {
        val rot = pose.R(); val c = pose.centre()
        val ox = c.x; val oy = c.y; val oz = -c.z
        val out = ByteArray(iw * ih)
        Parallel.forEach(ih) { v ->
            val rnd = Random((seed * 7919 + v).toLong())
            for (u in 0 until iw) {
                var acc = 0.0
                for (sj in 0..1) for (si in 0..1) {                                             // 2 x 2 samples per pixel
                    val xd = (u + (si + 0.5) / 2 - 0.5 - cam.cx) / cam.f; val yd = (v + (sj + 0.5) / 2 - 0.5 - cam.cy) / cam.f
                    var xn = xd; var yn = yd
                    for (q in 0 until 6) { val r2 = xn * xn + yn * yn; val dd = 1 + cam.k1 * r2 + cam.k2 * r2 * r2; xn = xd / dd; yn = yd / dd }
                    val dx = rot[0][0] * xn + rot[1][0] * yn + rot[2][0]; val dy = rot[0][1] * xn + rot[1][1] * yn + rot[2][1]; val dz = -(rot[0][2] * xn + rot[1][2] * yn + rot[2][2])
                    val dn = sqrt(dx * dx + dy * dy + dz * dz)
                    acc += shade(tpl, ox, oy, oz, dx / dn, dy / dn, dz / dn)
                }
                out[v * iw + u] = (acc / 4 + rnd.nextGaussian() * 1.5).coerceIn(0.0, 255.0).toInt().toByte()
            }
        }
        return out
    }

    /** Pixel of a z-up point, or null when it is outside the frame or hidden behind the box. */
    private fun see(pose: Pose6, p: Vec3, margin: Double, checkOcclusion: Boolean): DoubleArray? {
        val pr = cam.project(pose.toCamera(Vec3(p.x, p.y, -p.z))) ?: return null
        if (pr[0] < margin || pr[1] < margin || pr[0] > iw - 1 - margin || pr[1] > ih - 1 - margin) return null
        if (checkOcclusion) {
            val c = pose.centre(); val ox = c.x; val oy = c.y; val oz = -c.z
            val dx = p.x - ox; val dy = p.y - oy; val dz = p.z - oz; val dn = sqrt(dx * dx + dy * dy + dz * dz)
            val t = hit(ox, oy, oz, dx / dn, dy / dn, dz / dn)
            if (t >= 0 && t < dn - 1.0) return null
        }
        return pr
    }

    @Test fun syntheticScanGivesTheBoxBack() {
        assertTrue(OpenCVLoader.initLocal())
        val ctx = ApplicationProvider.getApplicationContext<android.content.Context>()
        val dir = File(ctx.cacheDir, "e2e").apply { deleteRecursively(); mkdirs() }
        val tpl = Pipeline.boardTemplate
        // 6 low, 6 middle (two of them with the phone turned sideways), 2 from above
        val views = listOf(doubleArrayOf(0.0, 24.0, 0.0), doubleArrayOf(60.0, 26.0, 0.0), doubleArrayOf(120.0, 23.0, 0.0), doubleArrayOf(180.0, 25.0, 0.0), doubleArrayOf(240.0, 27.0, 0.0), doubleArrayOf(300.0, 24.0, 0.0),
            doubleArrayOf(30.0, 48.0, 0.0), doubleArrayOf(90.0, 50.0, 90.0), doubleArrayOf(150.0, 47.0, 0.0), doubleArrayOf(210.0, 51.0, 0.0), doubleArrayOf(270.0, 49.0, 90.0), doubleArrayOf(330.0, 50.0, 0.0),
            doubleArrayOf(10.0, 74.0, 0.0), doubleArrayOf(190.0, 76.0, 0.0))
        val noise = Random(4711)
        val shots = ArrayList<Shot>(); val files = ArrayList<File>()
        val t0 = System.currentTimeMillis()
        for ((i, vw) in views.withIndex()) {
            val pose = lookAt(vw[0], vw[1], 330.0, vw[2])
            val gray = render(pose, tpl, i)
            val px = IntArray(iw * ih) { val g = gray[it].toInt() and 255; (0xFF shl 24) or (g shl 16) or (g shl 8) or g }
            val bm = Bitmap.createBitmap(px, iw, ih, Bitmap.Config.ARGB_8888)
            val f = File(dir, "shot${i + 1}.jpg"); f.outputStream().use { bm.compress(Bitmap.CompressFormat.JPEG, 95, it) }; bm.recycle()
            val shot = Shot(i + 1, "k${i + 1}"); shot.state = "done"; shot.w = iw; shot.h = ih; shot.fileRot = 0; shot.landscape = vw[2] != 0.0
            // what a detector would report: board corners and sticker corners, 0.15 px of noise each
            val ids = ArrayList<Int>(); val pts = ArrayList<Float>()
            for (id in 0 until 54) {
                val pr = see(pose, Vec3(((id % 6) + 1) * 25.0, ((id / 6) + 1) * 25.0, 0.0), 8.0, true) ?: continue
                ids.add(id); pts.add((pr[0] + noise.nextGaussian() * 0.15).toFloat()); pts.add((pr[1] + noise.nextGaussian() * 0.15).toFloat())
            }
            shot.board = BoardObs(ids.toIntArray(), pts.toFloatArray())
            val c = pose.centre(); val camUp = Vec3(c.x, c.y, -c.z)
            val seen = ArrayList<Sticker>()
            for (s in stickers) {
                val toCam = (camUp - s.c).unit()
                if (toCam.dot(s.n) < 0.35) continue
                val q = FloatArray(8); var ok = true
                for ((k, sg) in listOf(-1.0 to -1.0, 1.0 to -1.0, 1.0 to 1.0, -1.0 to 1.0).withIndex()) {
                    val pr = see(pose, s.c + s.u * (6.0 * sg.first) + s.v * (6.0 * sg.second), 4.0, false)
                    if (pr == null) { ok = false; break }
                    q[2 * k] = (pr[0] + noise.nextGaussian() * 0.15).toFloat(); q[2 * k + 1] = (pr[1] + noise.nextGaussian() * 0.15).toFloat()
                }
                if (ok) seen.add(Sticker(s.id, q))
            }
            shot.stickers = seen
            shots.add(shot); files.add(f)
        }
        Log.i("Scan3D", "e2e: rendered ${views.size} photos in ${System.currentTimeMillis() - t0} ms; board corners per photo ${shots.map { it.board!!.ids.size }}, stickers ${shots.map { it.stickers.size }}")

        val t1 = System.currentTimeMillis()
        val out = Pipeline.build(shots, files, true, null, 0.5, { _, _ -> }, { Log.i("Scan3D", "e2e: $it") }, null)
        val rep = out.report
        Log.i("Scan3D", "e2e: pipeline took ${System.currentTimeMillis() - t1} ms; hull %.2f x %.2f x %.2f mm, sigma0 %.3f px, sheet bend %.2f mm, f %.1f (truth %.1f)".format(
            rep.sizeMm[0], rep.sizeMm[1], rep.sizeMm[2], rep.sigma0, rep.sheetBendMm, rep.focalPx, cam.f))
        val fit = rep.fit
        assertNotNull("the clean model must be found for a rounded box", fit)
        val p = fit!!.p
        Log.i("Scan3D", "e2e: RESULT fit %.3f x %.3f x %.3f rc %.2f rz %.2f rb %.2f tilt %.2f deg | truth %.1f x %.1f x %.1f rc %.1f rz %.1f rb %.1f | within %.0f%%, median %.3f mm, edges %d, sigma W %.3f L %.3f H %.3f".format(
            p[RayFit.W], p[RayFit.L], p[RayFit.H], p[RayFit.RC], p[RayFit.RZ], p[RayFit.RB], fit.tiltDeg,
            truth[RayFit.W], truth[RayFit.L], truth[RayFit.H], truth[RayFit.RC], truth[RayFit.RZ], truth[RayFit.RB],
            100 * fit.within, fit.medianMm, fit.edges, fit.sigma?.get(RayFit.W) ?: -1.0, fit.sigma?.get(RayFit.L) ?: -1.0, fit.sigma?.get(RayFit.H) ?: -1.0))
        // one pixel of these quarter-size photos is 0.46 mm on the object
        assertTrue("W ${p[RayFit.W]}", abs(p[RayFit.W] - truth[RayFit.W]) < 0.4)
        assertTrue("L ${p[RayFit.L]}", abs(p[RayFit.L] - truth[RayFit.L]) < 0.4)
        assertTrue("H ${p[RayFit.H]}", abs(p[RayFit.H] - truth[RayFit.H]) < 0.4)
        assertTrue("tilt ${fit.tiltDeg}", fit.tiltDeg < 1.5)
        assertNotNull(out.clean)
        dir.deleteRecursively()
    }
}
