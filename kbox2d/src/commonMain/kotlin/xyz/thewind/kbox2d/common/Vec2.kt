package xyz.thewind.kbox2d.common

class Vec2(var x: Float = 0f, var y: Float = 0f) {
    constructor(toCopy: Vec2) : this(toCopy.x, toCopy.y)

    fun setZero() {
        x = 0f
        y = 0f
    }

    fun set(x: Float, y: Float): Vec2 {
        this.x = x
        this.y = y
        return this
    }

    fun set(v: Vec2): Vec2 {
        x = v.x
        y = v.y
        return this
    }

    fun add(v: Vec2): Vec2 = Vec2(x + v.x, y + v.y)

    fun sub(v: Vec2): Vec2 = Vec2(x - v.x, y - v.y)

    fun mul(a: Float): Vec2 = Vec2(x * a, y * a)

    fun negate(): Vec2 = Vec2(-x, -y)

    fun negateLocal(): Vec2 {
        x = -x
        y = -y
        return this
    }

    fun addLocal(v: Vec2): Vec2 {
        x += v.x
        y += v.y
        return this
    }

    fun addLocal(x: Float, y: Float): Vec2 {
        this.x += x
        this.y += y
        return this
    }

    fun subLocal(v: Vec2): Vec2 {
        x -= v.x
        y -= v.y
        return this
    }

    fun mulLocal(a: Float): Vec2 {
        x *= a
        y *= a
        return this
    }

    fun skew(): Vec2 = Vec2(-y, x)

    fun skew(out: Vec2) {
        out.x = -y
        out.y = x
    }

    fun length(): Float = MathUtils.sqrt(x * x + y * y)

    fun lengthSquared(): Float = x * x + y * y

    fun normalize(): Float {
        val length = length()
        if (length < Settings.EPSILON) {
            return 0f
        }
        val invLength = 1f / length
        x *= invLength
        y *= invLength
        return length
    }

    fun isValid(): Boolean = !x.isNaN() && !x.isInfinite() && !y.isNaN() && !y.isInfinite()

    fun abs(): Vec2 = Vec2(MathUtils.abs(x), MathUtils.abs(y))

    fun absLocal() {
        x = MathUtils.abs(x)
        y = MathUtils.abs(y)
    }

    fun clone(): Vec2 = Vec2(x, y)

    override fun toString(): String = "($x,$y)"

    override fun hashCode(): Int {
        var result = 1
        result = 31 * result + x.toBits()
        result = 31 * result + y.toBits()
        return result
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Vec2) return false
        return x.toBits() == other.x.toBits() && y.toBits() == other.y.toBits()
    }

    companion object {
        fun abs(a: Vec2): Vec2 = Vec2(MathUtils.abs(a.x), MathUtils.abs(a.y))

        fun absToOut(a: Vec2, out: Vec2) {
            out.x = MathUtils.abs(a.x)
            out.y = MathUtils.abs(a.y)
        }

        fun dot(a: Vec2, b: Vec2): Float = a.x * b.x + a.y * b.y

        fun cross(a: Vec2, b: Vec2): Float = a.x * b.y - a.y * b.x

        fun cross(a: Vec2, s: Float): Vec2 = Vec2(s * a.y, -s * a.x)

        fun crossToOut(a: Vec2, s: Float, out: Vec2) {
            val tempY = -s * a.x
            out.x = s * a.y
            out.y = tempY
        }

        fun crossToOutUnsafe(a: Vec2, s: Float, out: Vec2) {
            assert(out !== a)
            out.x = s * a.y
            out.y = -s * a.x
        }

        fun cross(s: Float, a: Vec2): Vec2 = Vec2(-s * a.y, s * a.x)

        fun crossToOut(s: Float, a: Vec2, out: Vec2) {
            val tempY = s * a.x
            out.x = -s * a.y
            out.y = tempY
        }

        fun crossToOutUnsafe(s: Float, a: Vec2, out: Vec2) {
            assert(out !== a)
            out.x = -s * a.y
            out.y = s * a.x
        }

        fun negateToOut(a: Vec2, out: Vec2) {
            out.x = -a.x
            out.y = -a.y
        }

        fun min(a: Vec2, b: Vec2): Vec2 = Vec2(
            if (a.x < b.x) a.x else b.x,
            if (a.y < b.y) a.y else b.y,
        )

        fun max(a: Vec2, b: Vec2): Vec2 = Vec2(
            if (a.x > b.x) a.x else b.x,
            if (a.y > b.y) a.y else b.y,
        )

        fun minToOut(a: Vec2, b: Vec2, out: Vec2) {
            out.x = if (a.x < b.x) a.x else b.x
            out.y = if (a.y < b.y) a.y else b.y
        }

        fun maxToOut(a: Vec2, b: Vec2, out: Vec2) {
            out.x = if (a.x > b.x) a.x else b.x
            out.y = if (a.y > b.y) a.y else b.y
        }
    }
}
