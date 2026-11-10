package ir.scan3d.app.engine

import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

/**
 * The outline a mesh has in a photo: every pixel whose centre is covered by a projected triangle.
 *
 * Used to look for the object's photographed edge next to the outline of the visual hull instead of next to one
 * photo's own mask. A single mask also holds everything that photo could not judge (plain squares, the far end of
 * a sheet seen at a flat angle, all that lies beyond the print) and the object often touches those regions; the
 * hull is what ALL photos agree on, with the loose pieces and the base skirt already removed.
 */
object Raster {
    /** mesh: z-up frame (mm), board plane z = 0. cam / pose: board frame (z into the sheet). Pixel centres at integers. */
    fun silhouette(mesh: Mesh, cam: Camera, pose: Pose6, w: Int, h: Int): BooleanArray {
        val rot = pose.R(); val t = pose.t
        val n = mesh.vertexCount; val v = mesh.verts
        val px = DoubleArray(n); val py = DoubleArray(n); val ok = BooleanArray(n)
        for (i in 0 until n) {
            val x = v[3 * i].toDouble(); val y = v[3 * i + 1].toDouble(); val z = -v[3 * i + 2].toDouble()
            val zc = rot[2][0] * x + rot[2][1] * y + rot[2][2] * z + t[2]
            if (zc <= 1e-6) continue
            val xn = (rot[0][0] * x + rot[0][1] * y + rot[0][2] * z + t[0]) / zc; val yn = (rot[1][0] * x + rot[1][1] * y + rot[1][2] * z + t[1]) / zc
            val r2 = xn * xn + yn * yn; val d = 1 + cam.k1 * r2 + cam.k2 * r2 * r2
            px[i] = cam.f * xn * d + cam.cx; py[i] = cam.f * yn * d + cam.cy; ok[i] = true
        }
        val out = BooleanArray(w * h)
        val tr = mesh.tris
        for (k in 0 until mesh.triangleCount) {
            val a = tr[3 * k]; val b = tr[3 * k + 1]; val c = tr[3 * k + 2]
            if (!ok[a] || !ok[b] || !ok[c]) continue
            val ax = px[a]; val ay = py[a]; val bx = px[b]; val by = py[b]; val cx = px[c]; val cy = py[c]
            val x0 = max(0, ceil(min(ax, min(bx, cx))).toInt()); val x1 = min(w - 1, floor(max(ax, max(bx, cx))).toInt())
            val y0 = max(0, ceil(min(ay, min(by, cy))).toInt()); val y1 = min(h - 1, floor(max(ay, max(by, cy))).toInt())
            if (x0 > x1 || y0 > y1) continue
            for (y in y0..y1) for (x in x0..x1) {
                // inclusive on every edge and for both windings: neighbouring triangles leave no pinholes
                val e0 = (bx - ax) * (y - ay) - (by - ay) * (x - ax)
                val e1 = (cx - bx) * (y - by) - (cy - by) * (x - bx)
                val e2 = (ax - cx) * (y - cy) - (ay - cy) * (x - cx)
                if ((e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0)) out[y * w + x] = true
            }
        }
        return out
    }
}
