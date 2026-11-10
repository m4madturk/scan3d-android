package ir.scan3d.app.engine

import java.io.File
import java.io.RandomAccessFile
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import java.nio.channels.FileChannel
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sqrt

/**
 * Reads a binary STL into what a renderer needs: one position and one shading normal per triangle corner.
 *
 * STL stores loose triangles with one normal each, which renders faceted. Here coincident corners are merged and a
 * corner gets the area-weighted mean normal of those faces around it that lie within the crease angle of its own
 * face: curved surfaces look curved, and real edges (the rim of the base) stay sharp.
 */
object Stl {
    /** data: x y z nx ny nz per corner (three corners per triangle), ready for a vertex buffer. */
    class Model(val data: FloatBuffer, val vertexCount: Int, val min: FloatArray, val max: FloatArray) {
        val triangles get() = vertexCount / 3
    }

    fun load(file: File, creaseDeg: Double = 40.0): Model? {
        val len = file.length()
        if (len < 84) return null
        RandomAccessFile(file, "r").use { raf ->
            val bb = raf.channel.map(FileChannel.MapMode.READ_ONLY, 0, len).order(ByteOrder.LITTLE_ENDIAN)
            val n = bb.getInt(80)
            if (n <= 0 || 84L + 50L * n > len) return null
            val pos = FloatArray(9 * n)
            for (t in 0 until n) { val o = 84 + 50 * t + 12; for (k in 0 until 9) pos[9 * t + k] = bb.getFloat(o + 4 * k) }
            return build(pos, creaseDeg)
        }
    }

    /** pos: 9 floats per triangle (three corners). */
    fun build(pos: FloatArray, creaseDeg: Double = 40.0): Model {
        val n = pos.size / 9
        val mn = floatArrayOf(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE); val mx = floatArrayOf(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE)
        for (i in 0 until 3 * n) for (a in 0 until 3) { val v = pos[3 * i + a]; if (v < mn[a]) mn[a] = v; if (v > mx[a]) mx[a] = v }
        // face normals: raw cross product (length = twice the area) and its unit vector
        val raw = FloatArray(3 * n); val unit = FloatArray(3 * n)
        for (t in 0 until n) {
            val o = 9 * t
            val ux = pos[o + 3] - pos[o]; val uy = pos[o + 4] - pos[o + 1]; val uz = pos[o + 5] - pos[o + 2]
            val vx = pos[o + 6] - pos[o]; val vy = pos[o + 7] - pos[o + 1]; val vz = pos[o + 8] - pos[o + 2]
            val nx = uy * vz - uz * vy; val ny = uz * vx - ux * vz; val nz = ux * vy - uy * vx
            raw[3 * t] = nx; raw[3 * t + 1] = ny; raw[3 * t + 2] = nz
            val l = sqrt(nx * nx + ny * ny + nz * nz)
            if (l > 0f) { unit[3 * t] = nx / l; unit[3 * t + 1] = ny / l; unit[3 * t + 2] = nz / l } else unit[3 * t + 2] = 1f
        }
        // merge coincident corners: positions quantised to a millionth of the model's extent (20 bits per axis)
        val ext = max(mx[0] - mn[0], max(mx[1] - mn[1], mx[2] - mn[2]))
        val q = if (ext > 0f) 1_000_000.0 / ext else 1.0
        val map = LongIntMap(1 shl 16); val vid = IntArray(3 * n); var nv = 0
        for (c in 0 until 3 * n) {
            val kx = Math.round((pos[3 * c] - mn[0]) * q); val ky = Math.round((pos[3 * c + 1] - mn[1]) * q); val kz = Math.round((pos[3 * c + 2] - mn[2]) * q)
            val key = (kx shl 40) or (ky shl 20) or kz
            var id = map.get(key)
            if (id < 0) { id = nv++; map.put(key, id) }
            vid[c] = id
        }
        // faces around every merged vertex (compressed rows)
        val start = IntArray(nv + 1)
        for (c in 0 until 3 * n) start[vid[c] + 1]++
        for (v in 0 until nv) start[v + 1] += start[v]
        val fill = start.copyOf(nv); val faces = IntArray(3 * n)
        for (t in 0 until n) for (k in 0 until 3) { val v = vid[3 * t + k]; faces[fill[v]++] = t }
        // a corner's normal: the area-weighted mean of the faces around its vertex that lie within the crease angle
        // of the corner's OWN face. (Comparing the mean of all faces with the own face is not enough: at a cube
        // corner the face with two triangles dominates the mean and would pass as "smooth".)
        val cosC = cos(Math.toRadians(creaseDeg)).toFloat()
        val out = ByteBuffer.allocateDirect(n * 18 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer()
        val rec = FloatArray(18)
        for (t in 0 until n) {
            val fx = unit[3 * t]; val fy = unit[3 * t + 1]; val fz = unit[3 * t + 2]
            for (k in 0 until 3) {
                val c = 3 * t + k; val v = vid[c]
                var ax = 0f; var ay = 0f; var az = 0f
                for (q in start[v] until start[v + 1]) {
                    val f = faces[q]
                    if (unit[3 * f] * fx + unit[3 * f + 1] * fy + unit[3 * f + 2] * fz >= cosC) { ax += raw[3 * f]; ay += raw[3 * f + 1]; az += raw[3 * f + 2] }
                }
                val l = sqrt(ax * ax + ay * ay + az * az)
                rec[6 * k] = pos[3 * c]; rec[6 * k + 1] = pos[3 * c + 1]; rec[6 * k + 2] = pos[3 * c + 2]
                if (l > 0f) { rec[6 * k + 3] = ax / l; rec[6 * k + 4] = ay / l; rec[6 * k + 5] = az / l } else { rec[6 * k + 3] = fx; rec[6 * k + 4] = fy; rec[6 * k + 5] = fz }
            }
            out.put(rec)
        }
        out.position(0)
        return Model(out, 3 * n, mn, mx)
    }
}
