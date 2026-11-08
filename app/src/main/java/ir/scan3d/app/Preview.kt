package ir.scan3d.app

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import ir.scan3d.app.engine.Mesh
import java.io.File
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/** Simple painter's-algorithm render of the mesh (two views) to a PNG, no GL needed. */
object Preview {
    fun render(mesh: Mesh, out: File, w: Int = 1200, h: Int = 620) {
        val bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        val c = Canvas(bmp); c.drawColor(Color.WHITE)
        drawView(c, mesh, 0, 0, w / 2, h, 25.0, -40.0)
        drawView(c, mesh, w / 2, 0, w / 2, h, 25.0, 140.0)
        out.outputStream().use { bmp.compress(Bitmap.CompressFormat.PNG, 90, it) }
        bmp.recycle()
    }

    fun bitmap(mesh: Mesh, w: Int, h: Int, elev: Double, azim: Double): Bitmap {
        val bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888); val c = Canvas(bmp); c.drawColor(Color.WHITE)
        drawView(c, mesh, 0, 0, w, h, elev, azim); return bmp
    }

    private fun drawView(c: Canvas, m: Mesh, x0: Int, y0: Int, w: Int, h: Int, elevDeg: Double, azimDeg: Double) {
        val b = m.bounds(); val cx = (b[0][0] + b[1][0]) / 2; val cy = (b[0][1] + b[1][1]) / 2; val cz = (b[0][2] + b[1][2]) / 2
        val dx = b[1][0] - b[0][0]; val dy = b[1][1] - b[0][1]; val dz = b[1][2] - b[0][2]
        val span = sqrt(dx * dx + dy * dy + dz * dz)          // bounding-sphere diameter: fits from every angle
        val s = 0.88 * minOf(w, h) / span
        val e = Math.toRadians(elevDeg); val a = Math.toRadians(azimDeg)
        // view axes
        val rx = doubleArrayOf(cos(a), sin(a), 0.0)
        val ry = doubleArrayOf(-sin(a) * sin(e), cos(a) * sin(e), cos(e))
        val rz = doubleArrayOf(-sin(a) * cos(e), cos(a) * cos(e), -sin(e))   // towards viewer
        val n = m.triangleCount
        val depth = DoubleArray(n); val order = (0 until n).toMutableList()
        val px = FloatArray(m.vertexCount); val py = FloatArray(m.vertexCount); val pz = DoubleArray(m.vertexCount)
        for (i in 0 until m.vertexCount) {
            val vx = m.verts[3 * i] - cx; val vy = m.verts[3 * i + 1] - cy; val vz = m.verts[3 * i + 2] - cz
            px[i] = (x0 + w / 2 + s * (vx * rx[0] + vy * rx[1] + vz * rx[2])).toFloat()
            py[i] = (y0 + h / 2 - s * (vx * ry[0] + vy * ry[1] + vz * ry[2])).toFloat()
            pz[i] = vx * rz[0] + vy * rz[1] + vz * rz[2]
        }
        for (t in 0 until n) depth[t] = (pz[m.tris[3 * t]] + pz[m.tris[3 * t + 1]] + pz[m.tris[3 * t + 2]]) / 3
        order.sortBy { depth[it] }
        // fill + 1 px stroke in the same colour: anti-aliased fills alone leave bright seams between dense triangles
        val paint = Paint(Paint.ANTI_ALIAS_FLAG); paint.style = Paint.Style.FILL_AND_STROKE; paint.strokeWidth = 1f; paint.strokeJoin = Paint.Join.ROUND
        val light = doubleArrayOf(0.3, -0.5, 0.8).let { val nn = sqrt(it[0] * it[0] + it[1] * it[1] + it[2] * it[2]); doubleArrayOf(it[0] / nn, it[1] / nn, it[2] / nn) }
        val path = Path()
        for (t in order) {
            val i0 = m.tris[3 * t]; val i1 = m.tris[3 * t + 1]; val i2 = m.tris[3 * t + 2]
            val ux = m.verts[3 * i1] - m.verts[3 * i0]; val uy = m.verts[3 * i1 + 1] - m.verts[3 * i0 + 1]; val uz = m.verts[3 * i1 + 2] - m.verts[3 * i0 + 2]
            val vx = m.verts[3 * i2] - m.verts[3 * i0]; val vy = m.verts[3 * i2 + 1] - m.verts[3 * i0 + 1]; val vz = m.verts[3 * i2 + 2] - m.verts[3 * i0 + 2]
            var nx = uy * vz - uz * vy; var ny = uz * vx - ux * vz; var nz = ux * vy - uy * vx
            val nn = sqrt((nx * nx + ny * ny + nz * nz).toDouble()); if (nn == 0.0) continue
            nx = (nx / nn).toFloat(); ny = (ny / nn).toFloat(); nz = (nz / nn).toFloat()
            if (nx * rz[0] + ny * rz[1] + nz * rz[2] < 0) continue           // back face
            val l = (0.45 + 0.55 * maxOf(0.0, nx * light[0] + ny * light[1] + nz * light[2])).coerceIn(0.2, 1.0)
            paint.color = Color.rgb((60 + 150 * l).toInt(), (120 + 120 * l).toInt(), (115 + 120 * l).toInt())
            path.reset(); path.moveTo(px[i0], py[i0]); path.lineTo(px[i1], py[i1]); path.lineTo(px[i2], py[i2]); path.close()
            c.drawPath(path, paint)
        }
        val tp = Paint(Paint.ANTI_ALIAS_FLAG); tp.color = Color.DKGRAY; tp.textSize = 22f
        c.drawText("%.1f x %.1f x %.1f mm".format(b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2]), x0 + 12f, y0 + h - 14f, tp)
    }
}
