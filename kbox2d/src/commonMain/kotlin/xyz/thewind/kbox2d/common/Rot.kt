package xyz.thewind.kbox2d.common

class Rot(var s: Float = 0f, var c: Float = 1f) {
    constructor(angle: Float) : this() {
        set(angle)
    }

    fun getSin(): Float = s

    fun getCos(): Float = c

    fun set(angle: Float): Rot {
        s = MathUtils.sin(angle)
        c = MathUtils.cos(angle)
        return this
    }

    fun set(other: Rot): Rot {
        s = other.s
        c = other.c
        return this
    }

    fun setIdentity(): Rot {
        s = 0f
        c = 1f
        return this
    }

    fun getAngle(): Float = MathUtils.atan2(s, c)

    fun getXAxis(xAxis: Vec2) {
        xAxis.set(c, s)
    }

    fun getYAxis(yAxis: Vec2) {
        yAxis.set(-s, c)
    }

    fun clone(): Rot = Rot().also {
        it.s = s
        it.c = c
    }

    override fun toString(): String = "Rot(s:$s, c:$c)"

    companion object {
        fun mul(q: Rot, r: Rot, out: Rot) {
            val tempc = q.c * r.c - q.s * r.s
            out.s = q.s * r.c + q.c * r.s
            out.c = tempc
        }

        fun mulUnsafe(q: Rot, r: Rot, out: Rot) {
            assert(r !== out)
            assert(q !== out)
            out.s = q.s * r.c + q.c * r.s
            out.c = q.c * r.c - q.s * r.s
        }

        fun mulTrans(q: Rot, r: Rot, out: Rot) {
            val tempc = q.c * r.c + q.s * r.s
            out.s = q.c * r.s - q.s * r.c
            out.c = tempc
        }

        fun mulTransUnsafe(q: Rot, r: Rot, out: Rot) {
            out.s = q.c * r.s - q.s * r.c
            out.c = q.c * r.c + q.s * r.s
        }

        fun mulToOut(q: Rot, v: Vec2, out: Vec2) {
            val tempy = q.s * v.x + q.c * v.y
            out.x = q.c * v.x - q.s * v.y
            out.y = tempy
        }

        fun mulToOutUnsafe(q: Rot, v: Vec2, out: Vec2) {
            out.x = q.c * v.x - q.s * v.y
            out.y = q.s * v.x + q.c * v.y
        }

        fun mulTrans(q: Rot, v: Vec2, out: Vec2) {
            val tempy = -q.s * v.x + q.c * v.y
            out.x = q.c * v.x + q.s * v.y
            out.y = tempy
        }

        fun mulTransUnsafe(q: Rot, v: Vec2, out: Vec2) {
            out.x = q.c * v.x + q.s * v.y
            out.y = -q.s * v.x + q.c * v.y
        }
    }
}
