package xyz.thewind.kbox2d.common

class Sweep {
    val localCenter: Vec2 = Vec2()
    val c0: Vec2 = Vec2()
    val c: Vec2 = Vec2()
    var a0: Float = 0f
    var a: Float = 0f
    var alpha0: Float = 0f

    fun normalize() {
        val d = MathUtils.TWOPI * MathUtils.floor(a0 / MathUtils.TWOPI)
        a0 -= d
        a -= d
    }

    fun set(other: Sweep): Sweep {
        localCenter.set(other.localCenter)
        c0.set(other.c0)
        c.set(other.c)
        a0 = other.a0
        a = other.a
        alpha0 = other.alpha0
        return this
    }

    fun getTransform(xf: Transform, beta: Float) {
        xf.p.x = (1.0f - beta) * c0.x + beta * c.x
        xf.p.y = (1.0f - beta) * c0.y + beta * c.y
        val angle = (1.0f - beta) * a0 + beta * a
        xf.q.set(angle)

        val q = xf.q
        xf.p.x -= q.c * localCenter.x - q.s * localCenter.y
        xf.p.y -= q.s * localCenter.x + q.c * localCenter.y
    }

    fun advance(alpha: Float) {
        assert(alpha0 < 1.0f)
        val beta = (alpha - alpha0) / (1.0f - alpha0)
        c0.x += beta * (c.x - c0.x)
        c0.y += beta * (c.y - c0.y)
        a0 += beta * (a - a0)
        alpha0 = alpha
    }

    override fun toString(): String = buildString {
        append("Sweep:\nlocalCenter: ").append(localCenter).append('\n')
        append("c0: ").append(c0).append(", c: ").append(c).append('\n')
        append("a0: ").append(a0).append(", a: ").append(a).append('\n')
        append("alpha0: ").append(alpha0)
    }
}
