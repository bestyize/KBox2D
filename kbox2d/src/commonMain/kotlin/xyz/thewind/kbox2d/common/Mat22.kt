package xyz.thewind.kbox2d.common

class Mat22 {
    val ex: Vec2
    val ey: Vec2

    constructor() {
        ex = Vec2()
        ey = Vec2()
    }

    constructor(c1: Vec2, c2: Vec2) {
        ex = c1.clone()
        ey = c2.clone()
    }

    constructor(exx: Float, col2x: Float, exy: Float, col2y: Float) {
        ex = Vec2(exx, exy)
        ey = Vec2(col2x, col2y)
    }

    fun set(m: Mat22): Mat22 {
        ex.x = m.ex.x
        ex.y = m.ex.y
        ey.x = m.ey.x
        ey.y = m.ey.y
        return this
    }

    fun set(exx: Float, col2x: Float, exy: Float, col2y: Float): Mat22 {
        ex.x = exx
        ex.y = exy
        ey.x = col2x
        ey.y = col2y
        return this
    }

    fun clone(): Mat22 = Mat22(ex, ey)

    fun set(angle: Float) {
        val c = MathUtils.cos(angle)
        val s = MathUtils.sin(angle)
        ex.x = c
        ey.x = -s
        ex.y = s
        ey.y = c
    }

    fun setIdentity() {
        ex.x = 1f
        ey.x = 0f
        ex.y = 0f
        ey.y = 1f
    }

    fun setZero() {
        ex.x = 0f
        ey.x = 0f
        ex.y = 0f
        ey.y = 0f
    }

    fun getAngle(): Float = MathUtils.atan2(ex.y, ex.x)

    fun set(c1: Vec2, c2: Vec2) {
        ex.x = c1.x
        ey.x = c2.x
        ex.y = c1.y
        ey.y = c2.y
    }

    fun invert(): Mat22 {
        val a = ex.x
        val b = ey.x
        val c = ex.y
        val d = ey.y
        var det = a * d - b * c
        if (det != 0f) det = 1f / det
        return Mat22(det * d, -det * b, -det * c, det * a)
    }

    fun invertLocal(): Mat22 {
        val a = ex.x
        val b = ey.x
        val c = ex.y
        val d = ey.y
        var det = a * d - b * c
        if (det != 0f) det = 1f / det
        ex.x = det * d
        ey.x = -det * b
        ex.y = -det * c
        ey.y = det * a
        return this
    }

    fun invertToOut(out: Mat22) {
        val a = ex.x
        val b = ey.x
        val c = ex.y
        val d = ey.y
        val det = 1f / (a * d - b * c)
        out.ex.x = det * d
        out.ey.x = -det * b
        out.ex.y = -det * c
        out.ey.y = det * a
    }

    fun abs(): Mat22 = Mat22(MathUtils.abs(ex.x), MathUtils.abs(ey.x), MathUtils.abs(ex.y), MathUtils.abs(ey.y))

    fun absLocal() {
        ex.absLocal()
        ey.absLocal()
    }

    fun mul(v: Vec2): Vec2 = Vec2(ex.x * v.x + ey.x * v.y, ex.y * v.x + ey.y * v.y)

    fun mulToOut(v: Vec2, out: Vec2) {
        val tempY = ex.y * v.x + ey.y * v.y
        out.x = ex.x * v.x + ey.x * v.y
        out.y = tempY
    }

    fun mulToOutUnsafe(v: Vec2, out: Vec2) {
        assert(v !== out)
        out.x = ex.x * v.x + ey.x * v.y
        out.y = ex.y * v.x + ey.y * v.y
    }

    fun mul(R: Mat22): Mat22 {
        val C = Mat22()
        C.ex.x = ex.x * R.ex.x + ey.x * R.ex.y
        C.ex.y = ex.y * R.ex.x + ey.y * R.ex.y
        C.ey.x = ex.x * R.ey.x + ey.x * R.ey.y
        C.ey.y = ex.y * R.ey.x + ey.y * R.ey.y
        return C
    }

    fun mulLocal(R: Mat22): Mat22 {
        mulToOut(R, this)
        return this
    }

    fun mulToOut(R: Mat22, out: Mat22) {
        val tempY1 = ex.y * R.ex.x + ey.y * R.ex.y
        val tempX1 = ex.x * R.ex.x + ey.x * R.ex.y
        out.ex.x = tempX1
        out.ex.y = tempY1
        val tempY2 = ex.y * R.ey.x + ey.y * R.ey.y
        val tempX2 = ex.x * R.ey.x + ey.x * R.ey.y
        out.ey.x = tempX2
        out.ey.y = tempY2
    }

    fun mulToOutUnsafe(R: Mat22, out: Mat22) {
        assert(out !== R)
        assert(out !== this)
        out.ex.x = ex.x * R.ex.x + ey.x * R.ex.y
        out.ex.y = ex.y * R.ex.x + ey.y * R.ex.y
        out.ey.x = ex.x * R.ey.x + ey.x * R.ey.y
        out.ey.y = ex.y * R.ey.x + ey.y * R.ey.y
    }

    fun mulTrans(B: Mat22): Mat22 {
        val C = Mat22()
        C.ex.x = Vec2.dot(ex, B.ex)
        C.ex.y = Vec2.dot(ey, B.ex)
        C.ey.x = Vec2.dot(ex, B.ey)
        C.ey.y = Vec2.dot(ey, B.ey)
        return C
    }

    fun mulTransLocal(B: Mat22): Mat22 {
        mulTransToOut(B, this)
        return this
    }

    fun mulTransToOut(B: Mat22, out: Mat22) {
        val x1 = ex.x * B.ex.x + ex.y * B.ex.y
        val y1 = ey.x * B.ex.x + ey.y * B.ex.y
        val x2 = ex.x * B.ey.x + ex.y * B.ey.y
        val y2 = ey.x * B.ey.x + ey.y * B.ey.y
        out.ex.x = x1
        out.ey.x = x2
        out.ex.y = y1
        out.ey.y = y2
    }

    fun mulTransToOutUnsafe(B: Mat22, out: Mat22) {
        assert(B !== out)
        assert(this !== out)
        out.ex.x = ex.x * B.ex.x + ex.y * B.ex.y
        out.ey.x = ex.x * B.ey.x + ex.y * B.ey.y
        out.ex.y = ey.x * B.ex.x + ey.y * B.ex.y
        out.ey.y = ey.x * B.ey.x + ey.y * B.ey.y
    }

    fun mulTrans(v: Vec2): Vec2 = Vec2(v.x * ex.x + v.y * ex.y, v.x * ey.x + v.y * ey.y)

    fun mulTransToOut(v: Vec2, out: Vec2) {
        val tempX = v.x * ex.x + v.y * ex.y
        out.y = v.x * ey.x + v.y * ey.y
        out.x = tempX
    }

    fun add(B: Mat22): Mat22 {
        val m = Mat22()
        m.ex.x = ex.x + B.ex.x
        m.ex.y = ex.y + B.ex.y
        m.ey.x = ey.x + B.ey.x
        m.ey.y = ey.y + B.ey.y
        return m
    }

    fun addLocal(B: Mat22): Mat22 {
        ex.x += B.ex.x
        ex.y += B.ex.y
        ey.x += B.ey.x
        ey.y += B.ey.y
        return this
    }

    fun solve(b: Vec2): Vec2 {
        val a11 = ex.x
        val a12 = ey.x
        val a21 = ex.y
        val a22 = ey.y
        var det = a11 * a22 - a12 * a21
        if (det != 0.0f) det = 1.0f / det
        return Vec2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x))
    }

    fun solveToOut(b: Vec2, out: Vec2) {
        val a11 = ex.x
        val a12 = ey.x
        val a21 = ex.y
        val a22 = ey.y
        var det = a11 * a22 - a12 * a21
        if (det != 0.0f) det = 1.0f / det
        val tempY = det * (a11 * b.y - a21 * b.x)
        out.x = det * (a22 * b.x - a12 * b.y)
        out.y = tempY
    }

    override fun toString(): String = "[${ex.x},${ey.x}]\n[${ex.y},${ey.y}]"

    override fun hashCode(): Int {
        var result = 1
        result = 31 * result + ex.hashCode()
        result = 31 * result + ey.hashCode()
        return result
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Mat22) return false
        return ex == other.ex && ey == other.ey
    }

    companion object {
        fun abs(R: Mat22): Mat22 = R.abs()

        fun absToOut(R: Mat22, out: Mat22) {
            out.ex.x = MathUtils.abs(R.ex.x)
            out.ex.y = MathUtils.abs(R.ex.y)
            out.ey.x = MathUtils.abs(R.ey.x)
            out.ey.y = MathUtils.abs(R.ey.y)
        }

        fun mul(R: Mat22, v: Vec2): Vec2 = Vec2(R.ex.x * v.x + R.ey.x * v.y, R.ex.y * v.x + R.ey.y * v.y)

        fun mulToOut(R: Mat22, v: Vec2, out: Vec2) {
            val tempY = R.ex.y * v.x + R.ey.y * v.y
            out.x = R.ex.x * v.x + R.ey.x * v.y
            out.y = tempY
        }

        fun mulToOutUnsafe(R: Mat22, v: Vec2, out: Vec2) {
            assert(v !== out)
            out.x = R.ex.x * v.x + R.ey.x * v.y
            out.y = R.ex.y * v.x + R.ey.y * v.y
        }

        fun mul(A: Mat22, B: Mat22): Mat22 = A.mul(B)

        fun mulToOut(A: Mat22, B: Mat22, out: Mat22) {
            A.mulToOut(B, out)
        }

        fun mulToOutUnsafe(A: Mat22, B: Mat22, out: Mat22) {
            A.mulToOutUnsafe(B, out)
        }

        fun mulTrans(R: Mat22, v: Vec2): Vec2 = Vec2(v.x * R.ex.x + v.y * R.ex.y, v.x * R.ey.x + v.y * R.ey.y)

        fun mulTransToOut(R: Mat22, v: Vec2, out: Vec2) {
            val outX = v.x * R.ex.x + v.y * R.ex.y
            out.y = v.x * R.ey.x + v.y * R.ey.y
            out.x = outX
        }

        fun mulTransToOutUnsafe(R: Mat22, v: Vec2, out: Vec2) {
            assert(out !== v)
            out.y = v.x * R.ey.x + v.y * R.ey.y
            out.x = v.x * R.ex.x + v.y * R.ex.y
        }

        fun mulTrans(A: Mat22, B: Mat22): Mat22 {
            val C = Mat22()
            C.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y
            C.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y
            C.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y
            C.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y
            return C
        }

        fun mulTransToOut(A: Mat22, B: Mat22, out: Mat22) {
            val x1 = A.ex.x * B.ex.x + A.ex.y * B.ex.y
            val y1 = A.ey.x * B.ex.x + A.ey.y * B.ex.y
            val x2 = A.ex.x * B.ey.x + A.ex.y * B.ey.y
            val y2 = A.ey.x * B.ey.x + A.ey.y * B.ey.y
            out.ex.x = x1
            out.ex.y = y1
            out.ey.x = x2
            out.ey.y = y2
        }

        fun mulTransToOutUnsafe(A: Mat22, B: Mat22, out: Mat22) {
            assert(A !== out)
            assert(B !== out)
            out.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y
            out.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y
            out.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y
            out.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y
        }

        fun createRotationalTransform(angle: Float): Mat22 = Mat22().also { createRotationalTransform(angle, it) }

        fun createRotationalTransform(angle: Float, out: Mat22) {
            val c = MathUtils.cos(angle)
            val s = MathUtils.sin(angle)
            out.ex.x = c
            out.ey.x = -s
            out.ex.y = s
            out.ey.y = c
        }

        fun createScaleTransform(scale: Float): Mat22 = Mat22().also { createScaleTransform(scale, it) }

        fun createScaleTransform(scale: Float, out: Mat22) {
            out.ex.x = scale
            out.ey.y = scale
        }
    }
}
