package xyz.thewind.kbox2d.common

class Mat33 {
    val ex: Vec3
    val ey: Vec3
    val ez: Vec3

    constructor() {
        ex = Vec3()
        ey = Vec3()
        ez = Vec3()
    }

    constructor(exx: Float, exy: Float, exz: Float, eyx: Float, eyy: Float, eyz: Float, ezx: Float, ezy: Float, ezz: Float) {
        ex = Vec3(exx, exy, exz)
        ey = Vec3(eyx, eyy, eyz)
        ez = Vec3(ezx, ezy, ezz)
    }

    constructor(argCol1: Vec3, argCol2: Vec3, argCol3: Vec3) {
        ex = argCol1.clone()
        ey = argCol2.clone()
        ez = argCol3.clone()
    }

    fun setZero() {
        ex.setZero()
        ey.setZero()
        ez.setZero()
    }

    fun set(exx: Float, exy: Float, exz: Float, eyx: Float, eyy: Float, eyz: Float, ezx: Float, ezy: Float, ezz: Float) {
        ex.x = exx
        ex.y = exy
        ex.z = exz
        ey.x = eyx
        ey.y = eyy
        ey.z = eyz
        ez.x = ezx
        ez.y = ezy
        ez.z = ezz
    }

    fun set(mat: Mat33) {
        ex.set(mat.ex)
        ey.set(mat.ey)
        ez.set(mat.ez)
    }

    fun setIdentity() {
        ex.x = 1f
        ex.y = 0f
        ex.z = 0f
        ey.x = 0f
        ey.y = 1f
        ey.z = 0f
        ez.x = 0f
        ez.y = 0f
        ez.z = 1f
    }

    fun solve22(b: Vec2): Vec2 = Vec2().also { solve22ToOut(b, it) }

    fun solve22ToOut(b: Vec2, out: Vec2) {
        val a11 = ex.x
        val a12 = ey.x
        val a21 = ex.y
        val a22 = ey.y
        var det = a11 * a22 - a12 * a21
        if (det != 0.0f) det = 1.0f / det
        out.x = det * (a22 * b.x - a12 * b.y)
        out.y = det * (a11 * b.y - a21 * b.x)
    }

    fun solve33(b: Vec3): Vec3 = Vec3().also { solve33ToOut(b, it) }

    fun solve33ToOut(b: Vec3, out: Vec3) {
        assert(b !== out)
        Vec3.crossToOutUnsafe(ey, ez, out)
        var det = Vec3.dot(ex, out)
        if (det != 0.0f) det = 1.0f / det
        Vec3.crossToOutUnsafe(ey, ez, out)
        val x = det * Vec3.dot(b, out)
        Vec3.crossToOutUnsafe(b, ez, out)
        val y = det * Vec3.dot(ex, out)
        Vec3.crossToOutUnsafe(ey, b, out)
        val z = det * Vec3.dot(ex, out)
        out.x = x
        out.y = y
        out.z = z
    }

    fun getInverse22(M: Mat33) {
        val a = ex.x
        val b = ey.x
        val c = ex.y
        val d = ey.y
        var det = a * d - b * c
        if (det != 0.0f) det = 1.0f / det
        M.ex.x = det * d
        M.ey.x = -det * b
        M.ex.z = 0.0f
        M.ex.y = -det * c
        M.ey.y = det * a
        M.ey.z = 0.0f
        M.ez.x = 0.0f
        M.ez.y = 0.0f
        M.ez.z = 0.0f
    }

    fun getSymInverse33(M: Mat33) {
        val bx = ey.y * ez.z - ey.z * ez.y
        val by = ey.z * ez.x - ey.x * ez.z
        val bz = ey.x * ez.y - ey.y * ez.x
        var det = ex.x * bx + ex.y * by + ex.z * bz
        if (det != 0.0f) det = 1.0f / det

        val a11 = ex.x
        val a12 = ey.x
        val a13 = ez.x
        val a22 = ey.y
        val a23 = ez.y
        val a33 = ez.z

        M.ex.x = det * (a22 * a33 - a23 * a23)
        M.ex.y = det * (a13 * a23 - a12 * a33)
        M.ex.z = det * (a12 * a23 - a13 * a22)
        M.ey.x = M.ex.y
        M.ey.y = det * (a11 * a33 - a13 * a13)
        M.ey.z = det * (a13 * a12 - a11 * a23)
        M.ez.x = M.ex.z
        M.ez.y = M.ey.z
        M.ez.z = det * (a11 * a22 - a12 * a12)
    }

    override fun hashCode(): Int {
        var result = 1
        result = 31 * result + ex.hashCode()
        result = 31 * result + ey.hashCode()
        result = 31 * result + ez.hashCode()
        return result
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Mat33) return false
        return ex == other.ex && ey == other.ey && ez == other.ez
    }

    companion object {
        val IDENTITY: Mat33 = Mat33(Vec3(1f, 0f, 0f), Vec3(0f, 1f, 0f), Vec3(0f, 0f, 1f))

        fun mul(A: Mat33, v: Vec3): Vec3 = Vec3(
            v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x,
            v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y,
            v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z,
        )

        fun mul22(A: Mat33, v: Vec2): Vec2 = Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y)

        fun mul22ToOut(A: Mat33, v: Vec2, out: Vec2) {
            val tempX = A.ex.x * v.x + A.ey.x * v.y
            out.y = A.ex.y * v.x + A.ey.y * v.y
            out.x = tempX
        }

        fun mul22ToOutUnsafe(A: Mat33, v: Vec2, out: Vec2) {
            assert(v !== out)
            out.y = A.ex.y * v.x + A.ey.y * v.y
            out.x = A.ex.x * v.x + A.ey.x * v.y
        }

        fun mulToOut(A: Mat33, v: Vec3, out: Vec3) {
            val tempY = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y
            val tempZ = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z
            out.x = v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x
            out.y = tempY
            out.z = tempZ
        }

        fun mulToOutUnsafe(A: Mat33, v: Vec3, out: Vec3) {
            assert(out !== v)
            out.x = v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x
            out.y = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y
            out.z = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z
        }

        fun setScaleTransform(scale: Float, out: Mat33) {
            out.ex.x = scale
            out.ey.y = scale
        }
    }
}
