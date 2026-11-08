package ir.scan3d.app.engine

import java.io.File
import java.io.OutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.abs

/** Indexed triangle mesh in mm. */
class Mesh(val verts: FloatArray, val tris: IntArray) {
    val vertexCount get() = verts.size / 3
    val triangleCount get() = tris.size / 3

    /**
     * Taubin smoothing (lambda/mu): removes the voxel staircase of marching cubes without shrinking the
     * model the way plain Laplacian smoothing does. Vertices on the base plane (z = minimum) stay put.
     */
    /** Merges coincident vertices (marching cubes emits per-triangle vertices) so neighbourhoods are real. */
    fun welded(): Mesh {
        val map = HashMap<Long, Int>(vertexCount)
        val remap = IntArray(vertexCount)
        val nv = ArrayList<Float>(vertexCount * 3)
        for (i in 0 until vertexCount) {
            val x = Math.round(verts[3 * i] * 2000f).toLong(); val y = Math.round(verts[3 * i + 1] * 2000f).toLong(); val z = Math.round(verts[3 * i + 2] * 2000f).toLong()
            val key = (x and 0x1FFFFF) or ((y and 0x1FFFFF) shl 21) or ((z and 0x1FFFFF) shl 42)
            val id = map.getOrPut(key) { nv.add(verts[3 * i]); nv.add(verts[3 * i + 1]); nv.add(verts[3 * i + 2]); nv.size / 3 - 1 }
            remap[i] = id
        }
        val nt = ArrayList<Int>(tris.size)
        for (t in 0 until triangleCount) { val a = remap[tris[3 * t]]; val b = remap[tris[3 * t + 1]]; val c = remap[tris[3 * t + 2]]; if (a != b && b != c && a != c) { nt.add(a); nt.add(b); nt.add(c) } }
        return Mesh(nv.toFloatArray(), nt.toIntArray())
    }

    fun taubin(iterations: Int = 10, lambda: Float = 0.5f, mu: Float = -0.53f): Mesh {
        val welded = welded()
        return welded.taubinWelded(iterations, lambda, mu)
    }

    private fun taubinWelded(iterations: Int, lambda: Float, mu: Float): Mesh {
        val n = vertexCount
        val adj = Array(n) { IntArray(0) }
        run {
            val lists = Array(n) { ArrayList<Int>(6) }
            for (t in 0 until triangleCount) { val a = tris[3 * t]; val b = tris[3 * t + 1]; val c = tris[3 * t + 2]
                lists[a].add(b); lists[a].add(c); lists[b].add(a); lists[b].add(c); lists[c].add(a); lists[c].add(b) }
            for (i in 0 until n) adj[i] = lists[i].distinct().toIntArray()
        }
        var zmin = Float.MAX_VALUE; for (i in 0 until n) zmin = minOf(zmin, verts[3 * i + 2])
        val fixed = BooleanArray(n) { verts[3 * it + 2] < zmin + 1e-4f }
        var cur = verts.copyOf(); var nxt = FloatArray(verts.size)
        for (it in 0 until iterations) for (factor in floatArrayOf(lambda, mu)) {
            for (i in 0 until n) {
                if (fixed[i] || adj[i].isEmpty()) { nxt[3 * i] = cur[3 * i]; nxt[3 * i + 1] = cur[3 * i + 1]; nxt[3 * i + 2] = cur[3 * i + 2]; continue }
                var sx = 0f; var sy = 0f; var sz = 0f
                for (j in adj[i]) { sx += cur[3 * j]; sy += cur[3 * j + 1]; sz += cur[3 * j + 2] }
                val k = adj[i].size
                nxt[3 * i] = cur[3 * i] + factor * (sx / k - cur[3 * i]); nxt[3 * i + 1] = cur[3 * i + 1] + factor * (sy / k - cur[3 * i + 1]); nxt[3 * i + 2] = cur[3 * i + 2] + factor * (sz / k - cur[3 * i + 2])
            }
            val tmp = cur; cur = nxt; nxt = tmp
        }
        return Mesh(cur, tris)
    }

    fun bounds(): Array<DoubleArray> {
        val lo = DoubleArray(3) { Double.MAX_VALUE }; val hi = DoubleArray(3) { -Double.MAX_VALUE }
        for (i in 0 until vertexCount) for (k in 0 until 3) { val v = verts[3 * i + k].toDouble(); if (v < lo[k]) lo[k] = v; if (v > hi[k]) hi[k] = v }
        return arrayOf(lo, hi)
    }

    /** Signed volume via divergence theorem (positive for outward-facing triangles). */
    fun volume(): Double {
        var v = 0.0
        for (t in 0 until triangleCount) {
            val a = tris[3 * t] * 3; val b = tris[3 * t + 1] * 3; val c = tris[3 * t + 2] * 3
            v += (verts[a] * (verts[b + 1] * verts[c + 2] - verts[b + 2] * verts[c + 1]) - verts[a + 1] * (verts[b] * verts[c + 2] - verts[b + 2] * verts[c]) + verts[a + 2] * (verts[b] * verts[c + 1] - verts[b + 1] * verts[c])) / 6.0
        }
        return v
    }

    fun translated(dx: Float, dy: Float, dz: Float): Mesh {
        val v = verts.copyOf(); for (i in 0 until vertexCount) { v[3 * i] += dx; v[3 * i + 1] += dy; v[3 * i + 2] += dz }
        return Mesh(v, tris)
    }

    /** Binary STL. */
    fun writeStl(out: OutputStream, name: String = "scan3d") {
        val header = ByteArray(80); val nb = name.toByteArray(); System.arraycopy(nb, 0, header, 0, minOf(80, nb.size))
        out.write(header)
        val bb = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN); bb.putInt(triangleCount); out.write(bb.array())
        val rec = ByteBuffer.allocate(50).order(ByteOrder.LITTLE_ENDIAN)
        for (t in 0 until triangleCount) {
            rec.clear()
            val a = tris[3 * t] * 3; val b = tris[3 * t + 1] * 3; val c = tris[3 * t + 2] * 3
            val ux = verts[b] - verts[a]; val uy = verts[b + 1] - verts[a + 1]; val uz = verts[b + 2] - verts[a + 2]
            val vx = verts[c] - verts[a]; val vy = verts[c + 1] - verts[a + 1]; val vz = verts[c + 2] - verts[a + 2]
            var nx = uy * vz - uz * vy; var ny = uz * vx - ux * vz; var nz = ux * vy - uy * vx
            val nn = Math.sqrt((nx * nx + ny * ny + nz * nz).toDouble()).toFloat(); if (nn > 0) { nx /= nn; ny /= nn; nz /= nn }
            rec.putFloat(nx).putFloat(ny).putFloat(nz)
            for (i in intArrayOf(a, b, c)) rec.putFloat(verts[i]).putFloat(verts[i + 1]).putFloat(verts[i + 2])
            rec.putShort(0)
            out.write(rec.array())
        }
        out.flush()
    }

    fun writeStl(file: File) = file.outputStream().buffered().use { writeStl(it) }
}

/**
 * Marching cubes on a Carve.Grid (surface at f = 0, object where f > 0), with the bottom closed
 * at the board plane. Outputs outward-facing triangles. Table-driven (standard 256-case tables).
 */
object MarchingCubes {
    fun run(g: Carve.Grid): Mesh {
        // pad the grid by one layer of "outside" and a mirrored bottom layer so the mesh is closed
        val nx = g.nx + 2; val ny = g.ny + 2; val nz = g.nz + 2
        val f = FloatArray(nx * ny * nz) { (-g.voxel).toFloat() }
        fun id(i: Int, j: Int, k: Int) = (i * ny + j) * nz + k
        for (i in 0 until g.nx) for (j in 0 until g.ny) for (k in 0 until g.nz) f[id(i + 1, j + 1, k + 1)] = g.f[g.idx(i, j, k)]
        for (i in 0 until g.nx) for (j in 0 until g.ny) f[id(i + 1, j + 1, 0)] = -abs(f[id(i + 1, j + 1, 1)])   // closes exactly at z = 0
        val verts = ArrayList<Float>(); val tris = ArrayList<Int>()
        val edgeCache = HashMap<Long, Int>()
        val vox = g.voxel
        fun pos(i: Int, j: Int, k: Int) = floatArrayOf((g.x0 + (i - 1) * vox).toFloat(), (g.y0 + (j - 1) * vox).toFloat(), (g.z0 + (k - 1) * vox).toFloat())
        fun vertexOnEdge(i0: Int, j0: Int, k0: Int, i1: Int, j1: Int, k1: Int): Int {
            val key = (id(i0, j0, k0).toLong() shl 32) or id(i1, j1, k1).toLong()
            edgeCache[key]?.let { return it }
            val a = f[id(i0, j0, k0)]; val b = f[id(i1, j1, k1)]
            val t = if (abs(a - b) < 1e-12f) 0.5f else (a / (a - b)).coerceIn(0f, 1f)
            val p0 = pos(i0, j0, k0); val p1 = pos(i1, j1, k1)
            verts.add(p0[0] + (p1[0] - p0[0]) * t); verts.add(p0[1] + (p1[1] - p0[1]) * t); verts.add(p0[2] + (p1[2] - p0[2]) * t)
            val idx = verts.size / 3 - 1; edgeCache[key] = idx; return idx
        }
        val cornerOff = arrayOf(intArrayOf(0, 0, 0), intArrayOf(1, 0, 0), intArrayOf(1, 1, 0), intArrayOf(0, 1, 0), intArrayOf(0, 0, 1), intArrayOf(1, 0, 1), intArrayOf(1, 1, 1), intArrayOf(0, 1, 1))
        val edgeCorners = arrayOf(intArrayOf(0, 1), intArrayOf(1, 2), intArrayOf(2, 3), intArrayOf(3, 0), intArrayOf(4, 5), intArrayOf(5, 6), intArrayOf(6, 7), intArrayOf(7, 4), intArrayOf(0, 4), intArrayOf(1, 5), intArrayOf(2, 6), intArrayOf(3, 7))
        for (i in 0 until nx - 1) for (j in 0 until ny - 1) for (k in 0 until nz - 1) {
            var cube = 0
            for (c in 0 until 8) if (f[id(i + cornerOff[c][0], j + cornerOff[c][1], k + cornerOff[c][2])] > 0) cube = cube or (1 shl c)
            if (cube == 0 || cube == 255) continue
            val tbl = TRI_TABLE[cube]
            var e = 0
            while (e < tbl.size && tbl[e] != -1) {
                val vIdx = IntArray(3)
                for (m in 0 until 3) {
                    val ec = edgeCorners[tbl[e + m]]
                    val c0 = cornerOff[ec[0]]; val c1 = cornerOff[ec[1]]
                    vIdx[m] = vertexOnEdge(i + c0[0], j + c0[1], k + c0[2], i + c1[0], j + c1[1], k + c1[2])
                }
                // table winding is for "inside = value < iso"; ours is inside = f > 0, so flip to face outward
                tris.add(vIdx[0]); tris.add(vIdx[2]); tris.add(vIdx[1])
                e += 3
            }
        }
        val m = Mesh(verts.toFloatArray(), tris.toIntArray())
        return if (m.volume() < 0) Mesh(m.verts, IntArray(m.tris.size) { val t = it / 3; val w = it % 3; m.tris[3 * t + intArrayOf(0, 2, 1)[w]] }) else m
    }

    // Standard marching cubes triangle table (Paul Bourke / Lorensen-Cline). Edges listed per case, -1 terminated.
    val TRI_TABLE: Array<IntArray> by lazy { parseTable() }

    private fun parseTable(): Array<IntArray> {
        val s = TABLE_TEXT.trim().split("\n")
        return Array(256) { i -> s[i].trim().split(Regex("\\s*,\\s*")).filter { it.isNotEmpty() }.map { it.toInt() }.toIntArray() }
    }

    private const val TABLE_TEXT = """
-1
0,8,3,-1
0,1,9,-1
1,8,3,9,8,1,-1
1,2,10,-1
0,8,3,1,2,10,-1
9,2,10,0,2,9,-1
2,8,3,2,10,8,10,9,8,-1
3,11,2,-1
0,11,2,8,11,0,-1
1,9,0,2,3,11,-1
1,11,2,1,9,11,9,8,11,-1
3,10,1,11,10,3,-1
0,10,1,0,8,10,8,11,10,-1
3,9,0,3,11,9,11,10,9,-1
9,8,10,10,8,11,-1
4,7,8,-1
4,3,0,7,3,4,-1
0,1,9,8,4,7,-1
4,1,9,4,7,1,7,3,1,-1
1,2,10,8,4,7,-1
3,4,7,3,0,4,1,2,10,-1
9,2,10,9,0,2,8,4,7,-1
2,10,9,2,9,7,2,7,3,7,9,4,-1
8,4,7,3,11,2,-1
11,4,7,11,2,4,2,0,4,-1
9,0,1,8,4,7,2,3,11,-1
4,7,11,9,4,11,9,11,2,9,2,1,-1
3,10,1,3,11,10,7,8,4,-1
1,11,10,1,4,11,1,0,4,7,11,4,-1
4,7,8,9,0,11,9,11,10,11,0,3,-1
4,7,11,4,11,9,9,11,10,-1
9,5,4,-1
9,5,4,0,8,3,-1
0,5,4,1,5,0,-1
8,5,4,8,3,5,3,1,5,-1
1,2,10,9,5,4,-1
3,0,8,1,2,10,4,9,5,-1
5,2,10,5,4,2,4,0,2,-1
2,10,5,3,2,5,3,5,4,3,4,8,-1
9,5,4,2,3,11,-1
0,11,2,0,8,11,4,9,5,-1
0,5,4,0,1,5,2,3,11,-1
2,1,5,2,5,8,2,8,11,4,8,5,-1
10,3,11,10,1,3,9,5,4,-1
4,9,5,0,8,1,8,10,1,8,11,10,-1
5,4,0,5,0,11,5,11,10,11,0,3,-1
5,4,8,5,8,10,10,8,11,-1
9,7,8,5,7,9,-1
9,3,0,9,5,3,5,7,3,-1
0,7,8,0,1,7,1,5,7,-1
1,5,3,3,5,7,-1
9,7,8,9,5,7,10,1,2,-1
10,1,2,9,5,0,5,3,0,5,7,3,-1
8,0,2,8,2,5,8,5,7,10,5,2,-1
2,10,5,2,5,3,3,5,7,-1
7,9,5,7,8,9,3,11,2,-1
9,5,7,9,7,2,9,2,0,2,7,11,-1
2,3,11,0,1,8,1,7,8,1,5,7,-1
11,2,1,11,1,7,7,1,5,-1
9,5,8,8,5,7,10,1,3,10,3,11,-1
5,7,0,5,0,9,7,11,0,1,0,10,11,10,0,-1
11,10,0,11,0,3,10,5,0,8,0,7,5,7,0,-1
11,10,5,7,11,5,-1
10,6,5,-1
0,8,3,5,10,6,-1
9,0,1,5,10,6,-1
1,8,3,1,9,8,5,10,6,-1
1,6,5,2,6,1,-1
1,6,5,1,2,6,3,0,8,-1
9,6,5,9,0,6,0,2,6,-1
5,9,8,5,8,2,5,2,6,3,2,8,-1
2,3,11,10,6,5,-1
11,0,8,11,2,0,10,6,5,-1
0,1,9,2,3,11,5,10,6,-1
5,10,6,1,9,2,9,11,2,9,8,11,-1
6,3,11,6,5,3,5,1,3,-1
0,8,11,0,11,5,0,5,1,5,11,6,-1
3,11,6,0,3,6,0,6,5,0,5,9,-1
6,5,9,6,9,11,11,9,8,-1
5,10,6,4,7,8,-1
4,3,0,4,7,3,6,5,10,-1
1,9,0,5,10,6,8,4,7,-1
10,6,5,1,9,7,1,7,3,7,9,4,-1
6,1,2,6,5,1,4,7,8,-1
1,2,5,5,2,6,3,0,4,3,4,7,-1
8,4,7,9,0,5,0,6,5,0,2,6,-1
7,3,9,7,9,4,3,2,9,5,9,6,2,6,9,-1
3,11,2,7,8,4,10,6,5,-1
5,10,6,4,7,2,4,2,0,2,7,11,-1
0,1,9,4,7,8,2,3,11,5,10,6,-1
9,2,1,9,11,2,9,4,11,7,11,4,5,10,6,-1
8,4,7,3,11,5,3,5,1,5,11,6,-1
5,1,11,5,11,6,1,0,11,7,11,4,0,4,11,-1
0,5,9,0,6,5,0,3,6,11,6,3,8,4,7,-1
6,5,9,6,9,11,4,7,9,7,11,9,-1
10,4,9,6,4,10,-1
4,10,6,4,9,10,0,8,3,-1
10,0,1,10,6,0,6,4,0,-1
8,3,1,8,1,6,8,6,4,6,1,10,-1
1,4,9,1,2,4,2,6,4,-1
3,0,8,1,2,9,2,4,9,2,6,4,-1
0,2,4,4,2,6,-1
8,3,2,8,2,4,4,2,6,-1
10,4,9,10,6,4,11,2,3,-1
0,8,2,2,8,11,4,9,10,4,10,6,-1
3,11,2,0,1,6,0,6,4,6,1,10,-1
6,4,1,6,1,10,4,8,1,2,1,11,8,11,1,-1
9,6,4,9,3,6,9,1,3,11,6,3,-1
8,11,1,8,1,0,11,6,1,9,1,4,6,4,1,-1
3,11,6,3,6,0,0,6,4,-1
6,4,8,11,6,8,-1
7,10,6,7,8,10,8,9,10,-1
0,7,3,0,10,7,0,9,10,6,7,10,-1
10,6,7,1,10,7,1,7,8,1,8,0,-1
10,6,7,10,7,1,1,7,3,-1
1,2,6,1,6,8,1,8,9,8,6,7,-1
2,6,9,2,9,1,6,7,9,0,9,3,7,3,9,-1
7,8,0,7,0,6,6,0,2,-1
7,3,2,6,7,2,-1
2,3,11,10,6,8,10,8,9,8,6,7,-1
2,0,7,2,7,11,0,9,7,6,7,10,9,10,7,-1
1,8,0,1,7,8,1,10,7,6,7,10,2,3,11,-1
11,2,1,11,1,7,10,6,1,6,7,1,-1
8,9,6,8,6,7,9,1,6,11,6,3,1,3,6,-1
0,9,1,11,6,7,-1
7,8,0,7,0,6,3,11,0,11,6,0,-1
7,11,6,-1
7,6,11,-1
3,0,8,11,7,6,-1
0,1,9,11,7,6,-1
8,1,9,8,3,1,11,7,6,-1
10,1,2,6,11,7,-1
1,2,10,3,0,8,6,11,7,-1
2,9,0,2,10,9,6,11,7,-1
6,11,7,2,10,3,10,8,3,10,9,8,-1
7,2,3,6,2,7,-1
7,0,8,7,6,0,6,2,0,-1
2,7,6,2,3,7,0,1,9,-1
1,6,2,1,8,6,1,9,8,8,7,6,-1
10,7,6,10,1,7,1,3,7,-1
10,7,6,1,7,10,1,8,7,1,0,8,-1
0,3,7,0,7,10,0,10,9,6,10,7,-1
7,6,10,7,10,8,8,10,9,-1
6,8,4,11,8,6,-1
3,6,11,3,0,6,0,4,6,-1
8,6,11,8,4,6,9,0,1,-1
9,4,6,9,6,3,9,3,1,11,3,6,-1
6,8,4,6,11,8,2,10,1,-1
1,2,10,3,0,11,0,6,11,0,4,6,-1
4,11,8,4,6,11,0,2,9,2,10,9,-1
10,9,3,10,3,2,9,4,3,11,3,6,4,6,3,-1
8,2,3,8,4,2,4,6,2,-1
0,4,2,4,6,2,-1
1,9,0,2,3,4,2,4,6,4,3,8,-1
1,9,4,1,4,2,2,4,6,-1
8,1,3,8,6,1,8,4,6,6,10,1,-1
10,1,0,10,0,6,6,0,4,-1
4,6,3,4,3,8,6,10,3,0,3,9,10,9,3,-1
10,9,4,6,10,4,-1
4,9,5,7,6,11,-1
0,8,3,4,9,5,11,7,6,-1
5,0,1,5,4,0,7,6,11,-1
11,7,6,8,3,4,3,5,4,3,1,5,-1
9,5,4,10,1,2,7,6,11,-1
6,11,7,1,2,10,0,8,3,4,9,5,-1
7,6,11,5,4,10,4,2,10,4,0,2,-1
3,4,8,3,5,4,3,2,5,10,5,2,11,7,6,-1
7,2,3,7,6,2,5,4,9,-1
9,5,4,0,8,6,0,6,2,6,8,7,-1
3,6,2,3,7,6,1,5,0,5,4,0,-1
6,2,8,6,8,7,2,1,8,4,8,5,1,5,8,-1
9,5,4,10,1,6,1,7,6,1,3,7,-1
1,6,10,1,7,6,1,0,7,8,7,0,9,5,4,-1
4,0,10,4,10,5,0,3,10,6,10,7,3,7,10,-1
7,6,10,7,10,8,5,4,10,4,8,10,-1
6,9,5,6,11,9,11,8,9,-1
3,6,11,0,6,3,0,5,6,0,9,5,-1
0,11,8,0,5,11,0,1,5,5,6,11,-1
6,11,3,6,3,5,5,3,1,-1
1,2,10,9,5,11,9,11,8,11,5,6,-1
0,11,3,0,6,11,0,9,6,5,6,9,1,2,10,-1
11,8,5,11,5,6,8,0,5,10,5,2,0,2,5,-1
6,11,3,6,3,5,2,10,3,10,5,3,-1
5,8,9,5,2,8,5,6,2,3,8,2,-1
9,5,6,9,6,0,0,6,2,-1
1,5,8,1,8,0,5,6,8,3,8,2,6,2,8,-1
1,5,6,2,1,6,-1
1,3,6,1,6,10,3,8,6,5,6,9,8,9,6,-1
10,1,0,10,0,6,9,5,0,5,6,0,-1
0,3,8,5,6,10,-1
10,5,6,-1
11,5,10,7,5,11,-1
11,5,10,11,7,5,8,3,0,-1
5,11,7,5,10,11,1,9,0,-1
10,7,5,10,11,7,9,8,1,8,3,1,-1
11,1,2,11,7,1,7,5,1,-1
0,8,3,1,2,7,1,7,5,7,2,11,-1
9,7,5,9,2,7,9,0,2,2,11,7,-1
7,5,2,7,2,11,5,9,2,3,2,8,9,8,2,-1
2,5,10,2,3,5,3,7,5,-1
8,2,0,8,5,2,8,7,5,10,2,5,-1
9,0,1,5,10,3,5,3,7,3,10,2,-1
9,8,2,9,2,1,8,7,2,10,2,5,7,5,2,-1
1,3,5,3,7,5,-1
0,8,7,0,7,1,1,7,5,-1
9,0,3,9,3,5,5,3,7,-1
9,8,7,5,9,7,-1
5,8,4,5,10,8,10,11,8,-1
5,0,4,5,11,0,5,10,11,11,3,0,-1
0,1,9,8,4,10,8,10,11,10,4,5,-1
10,11,4,10,4,5,11,3,4,9,4,1,3,1,4,-1
2,5,1,2,8,5,2,11,8,4,5,8,-1
0,4,11,0,11,3,4,5,11,2,11,1,5,1,11,-1
0,2,5,0,5,9,2,11,5,4,5,8,11,8,5,-1
9,4,5,2,11,3,-1
2,5,10,3,5,2,3,4,5,3,8,4,-1
5,10,2,5,2,4,4,2,0,-1
3,10,2,3,5,10,3,8,5,4,5,8,0,1,9,-1
5,10,2,5,2,4,1,9,2,9,4,2,-1
8,4,5,8,5,3,3,5,1,-1
0,4,5,1,0,5,-1
8,4,5,8,5,3,9,0,5,0,3,5,-1
9,4,5,-1
4,11,7,4,9,11,9,10,11,-1
0,8,3,4,9,7,9,11,7,9,10,11,-1
1,10,11,1,11,4,1,4,0,7,4,11,-1
3,1,4,3,4,8,1,10,4,7,4,11,10,11,4,-1
4,11,7,9,11,4,9,2,11,9,1,2,-1
9,7,4,9,11,7,9,1,11,2,11,1,0,8,3,-1
11,7,4,11,4,2,2,4,0,-1
11,7,4,11,4,2,8,3,4,3,2,4,-1
2,9,10,2,7,9,2,3,7,7,4,9,-1
9,10,7,9,7,4,10,2,7,8,7,0,2,0,7,-1
3,7,10,3,10,2,7,4,10,1,10,0,4,0,10,-1
1,10,2,8,7,4,-1
4,9,1,4,1,7,7,1,3,-1
4,9,1,4,1,7,0,8,1,8,7,1,-1
4,0,3,7,4,3,-1
4,8,7,-1
9,10,8,10,11,8,-1
3,0,9,3,9,11,11,9,10,-1
0,1,10,0,10,8,8,10,11,-1
3,1,10,11,3,10,-1
1,2,11,1,11,9,9,11,8,-1
3,0,9,3,9,11,1,2,9,2,11,9,-1
0,2,11,8,0,11,-1
3,2,11,-1
2,3,8,2,8,10,10,8,9,-1
9,10,2,0,9,2,-1
2,3,8,2,8,10,0,1,8,1,10,8,-1
1,10,2,-1
1,3,8,9,1,8,-1
0,9,1,-1
0,3,8,-1
-1
"""
}
