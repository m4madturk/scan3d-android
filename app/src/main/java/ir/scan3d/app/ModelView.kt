package ir.scan3d.app

import android.content.Context
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.util.AttributeSet
import android.util.Log
import android.view.GestureDetector
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import ir.scan3d.app.engine.Stl
import java.nio.ByteBuffer
import java.nio.ByteOrder
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.sqrt

/**
 * The finished model, to turn around with a finger: drag to rotate, pinch to zoom, double tap to reset.
 * Orthographic (no perspective, like a CAD view) on a 10 mm grid, so sizes can be judged by eye.
 */
class ModelView @JvmOverloads constructor(ctx: Context, attrs: AttributeSet? = null) : GLSurfaceView(ctx, attrs), GLSurfaceView.Renderer {
    @Volatile private var az = AZ0
    @Volatile private var el = EL0
    @Volatile private var zoom = 1f
    @Volatile private var model: Stl.Model? = null
    @Volatile private var tint = floatArrayOf(0.42f, 0.80f, 0.77f)
    /** Background colour (r, g, b in 0..1): the card the view sits on. Set before the first frame. */
    @Volatile var background3 = floatArrayOf(0.086f, 0.122f, 0.110f)
    var onFailure: (() -> Unit)? = null          // GL could not be set up: the caller falls back to a picture

    private var uploaded: Stl.Model? = null
    private var prog = 0; private var lineProg = 0
    private var vbo = 0; private var gridVbo = 0; private var gridCount = 0
    private var w = 1; private var h = 1
    private val mModel = FloatArray(16); private val mProj = FloatArray(16); private val mMvp = FloatArray(16)
    private var lx = 0f; private var ly = 0f; private var dragging = false

    private val scaler = ScaleGestureDetector(ctx, object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
        override fun onScale(d: ScaleGestureDetector): Boolean { zoom = (zoom * d.scaleFactor).coerceIn(0.5f, 8f); requestRender(); return true }
    })
    private val taps = GestureDetector(ctx, object : GestureDetector.SimpleOnGestureListener() {
        override fun onDoubleTap(e: MotionEvent): Boolean { az = AZ0; el = EL0; zoom = 1f; requestRender(); return true }
    })

    init {
        setEGLContextClientVersion(2)
        setRenderer(this)
        renderMode = RENDERMODE_WHEN_DIRTY
        preserveEGLContextOnPause = true
    }

    /** Shows a model (null clears the view). r, g, b: its colour. */
    fun show(m: Stl.Model?, r: Float = 0.42f, g: Float = 0.80f, b: Float = 0.77f) { tint = floatArrayOf(r, g, b); model = m; requestRender() }

    override fun onTouchEvent(e: MotionEvent): Boolean {
        scaler.onTouchEvent(e); taps.onTouchEvent(e)
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> { parent?.requestDisallowInterceptTouchEvent(true); lx = e.x; ly = e.y; dragging = true }
            MotionEvent.ACTION_POINTER_DOWN -> dragging = false
            MotionEvent.ACTION_MOVE -> if (dragging && e.pointerCount == 1 && !scaler.isInProgress) {
                az += (e.x - lx) * 0.45f; el = (el + (e.y - ly) * 0.45f).coerceIn(-15f, 89f); lx = e.x; ly = e.y; requestRender()
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> { dragging = false; parent?.requestDisallowInterceptTouchEvent(false) }
        }
        return true
    }

    // ---------------- renderer (GL thread) ----------------
    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        prog = program(VS, FS); lineProg = program(LINE_VS, LINE_FS)
        uploaded = null; vbo = 0; gridVbo = 0
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        GLES20.glDisable(GLES20.GL_CULL_FACE)          // scans are not guaranteed to be consistently wound
        if (prog == 0 || lineProg == 0) post { onFailure?.invoke() }
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) { w = max(1, width); h = max(1, height); GLES20.glViewport(0, 0, w, h) }

    override fun onDrawFrame(gl: GL10?) {
        val bg = background3
        GLES20.glClearColor(bg[0], bg[1], bg[2], 1f)
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        val m = model
        if (m !== uploaded) upload(m)
        if (m == null || prog == 0 || vbo == 0) return
        val cx = (m.min[0] + m.max[0]) / 2; val cy = (m.min[1] + m.max[1]) / 2; val cz = (m.min[2] + m.max[2]) / 2
        val dx = m.max[0] - m.min[0]; val dy = m.max[1] - m.min[1]; val dz = m.max[2] - m.min[2]
        val radius = max(1f, 0.5f * sqrt(dx * dx + dy * dy + dz * dz))
        // model is z-up: turn it about z (azimuth), then tip the z axis towards the screen's y (elevation)
        Matrix.setIdentityM(mModel, 0)
        Matrix.rotateM(mModel, 0, el - 90f, 1f, 0f, 0f)
        Matrix.rotateM(mModel, 0, az, 0f, 0f, 1f)
        Matrix.translateM(mModel, 0, -cx, -cy, -cz)
        val half = radius * 1.12f / zoom
        val aspect = w.toFloat() / h
        if (aspect >= 1f) Matrix.orthoM(mProj, 0, -half * aspect, half * aspect, -half, half, -4f * radius, 4f * radius)
        else Matrix.orthoM(mProj, 0, -half, half, -half / aspect, half / aspect, -4f * radius, 4f * radius)
        Matrix.multiplyMM(mMvp, 0, mProj, 0, mModel, 0)

        // 10 mm grid on the plane the object stands on
        if (lineProg != 0 && gridVbo != 0) {
            GLES20.glUseProgram(lineProg)
            GLES20.glUniformMatrix4fv(GLES20.glGetUniformLocation(lineProg, "uMvp"), 1, false, mMvp, 0)
            GLES20.glUniform3f(GLES20.glGetUniformLocation(lineProg, "uColor"), bg[0] + 0.13f, bg[1] + 0.15f, bg[2] + 0.14f)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, gridVbo)
            val a = GLES20.glGetAttribLocation(lineProg, "aPos")
            GLES20.glEnableVertexAttribArray(a); GLES20.glVertexAttribPointer(a, 3, GLES20.GL_FLOAT, false, 12, 0)
            GLES20.glLineWidth(2f)
            GLES20.glDrawArrays(GLES20.GL_LINES, 0, gridCount)
            GLES20.glDisableVertexAttribArray(a)
        }
        GLES20.glUseProgram(prog)
        GLES20.glUniformMatrix4fv(GLES20.glGetUniformLocation(prog, "uMvp"), 1, false, mMvp, 0)
        GLES20.glUniformMatrix4fv(GLES20.glGetUniformLocation(prog, "uModel"), 1, false, mModel, 0)
        val t = tint
        GLES20.glUniform3f(GLES20.glGetUniformLocation(prog, "uColor"), t[0], t[1], t[2])
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo)
        val aPos = GLES20.glGetAttribLocation(prog, "aPos"); val aNor = GLES20.glGetAttribLocation(prog, "aNor")
        GLES20.glEnableVertexAttribArray(aPos); GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 24, 0)
        GLES20.glEnableVertexAttribArray(aNor); GLES20.glVertexAttribPointer(aNor, 3, GLES20.GL_FLOAT, false, 24, 12)
        GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, m.vertexCount)
        GLES20.glDisableVertexAttribArray(aPos); GLES20.glDisableVertexAttribArray(aNor)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
    }

    private fun upload(m: Stl.Model?) {
        val ids = IntArray(2)
        if (vbo != 0) { ids[0] = vbo; ids[1] = gridVbo; GLES20.glDeleteBuffers(2, ids, 0); vbo = 0; gridVbo = 0 }
        uploaded = m
        if (m == null) return
        GLES20.glGenBuffers(2, ids, 0); vbo = ids[0]; gridVbo = ids[1]
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo)
        m.data.position(0)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, m.vertexCount * 24, m.data, GLES20.GL_STATIC_DRAW)
        // grid lines every 10 mm around the model, on its lowest plane
        val cx = (m.min[0] + m.max[0]) / 2; val cy = (m.min[1] + m.max[1]) / 2; val z = m.min[2] - 0.05f
        val e = (ceil((max(m.max[0] - m.min[0], m.max[1] - m.min[1]) / 2 + 12f) / 10f) * 10f).coerceIn(20f, 400f)
        val nLines = (2 * e / 10f).toInt() + 1
        val buf = ByteBuffer.allocateDirect(nLines * 2 * 2 * 3 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer()
        for (i in 0 until nLines) {
            val o = -e + 10f * i
            buf.put(cx + o).put(cy - e).put(z).put(cx + o).put(cy + e).put(z)
            buf.put(cx - e).put(cy + o).put(z).put(cx + e).put(cy + o).put(z)
        }
        buf.position(0); gridCount = nLines * 4
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, gridVbo)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, gridCount * 12, buf, GLES20.GL_STATIC_DRAW)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
    }

    private fun program(vs: String, fs: String): Int {
        fun shader(type: Int, src: String): Int {
            val s = GLES20.glCreateShader(type); GLES20.glShaderSource(s, src); GLES20.glCompileShader(s)
            val ok = IntArray(1); GLES20.glGetShaderiv(s, GLES20.GL_COMPILE_STATUS, ok, 0)
            if (ok[0] == 0) { Log.w("Scan3D", "shader: " + GLES20.glGetShaderInfoLog(s)); GLES20.glDeleteShader(s); return 0 }
            return s
        }
        val v = shader(GLES20.GL_VERTEX_SHADER, vs); val f = shader(GLES20.GL_FRAGMENT_SHADER, fs)
        if (v == 0 || f == 0) return 0
        val p = GLES20.glCreateProgram(); GLES20.glAttachShader(p, v); GLES20.glAttachShader(p, f); GLES20.glLinkProgram(p)
        val ok = IntArray(1); GLES20.glGetProgramiv(p, GLES20.GL_LINK_STATUS, ok, 0)
        if (ok[0] == 0) { Log.w("Scan3D", "program: " + GLES20.glGetProgramInfoLog(p)); GLES20.glDeleteProgram(p); return 0 }
        return p
    }

    companion object {
        private const val AZ0 = 32f
        private const val EL0 = 26f
        private const val VS = """
            attribute vec3 aPos; attribute vec3 aNor;
            uniform mat4 uMvp; uniform mat4 uModel;
            varying vec3 vN;
            void main() { vN = (uModel * vec4(aNor, 0.0)).xyz; gl_Position = uMvp * vec4(aPos, 1.0); }"""
        // a key light from the upper left, a weak fill from the right, a rim so the silhouette reads on a dark card;
        // normals that face away are flipped (two-sided), the view direction being -z
        private const val FS = """
            precision mediump float;
            varying vec3 vN; uniform vec3 uColor;
            void main() {
                vec3 n = normalize(vN); if (n.z < 0.0) n = -n;
                float key = max(dot(n, normalize(vec3(-0.45, 0.60, 0.66))), 0.0);
                float fill = max(dot(n, normalize(vec3(0.70, 0.10, 0.50))), 0.0);
                float rim = pow(1.0 - n.z, 3.0);
                vec3 c = uColor * (0.20 + 0.66 * key + 0.20 * fill) + vec3(0.16) * rim;
                gl_FragColor = vec4(c, 1.0);
            }"""
        private const val LINE_VS = "attribute vec3 aPos; uniform mat4 uMvp; void main() { gl_Position = uMvp * vec4(aPos, 1.0); }"
        private const val LINE_FS = "precision mediump float; uniform vec3 uColor; void main() { gl_FragColor = vec4(uColor, 1.0); }"
    }
}
