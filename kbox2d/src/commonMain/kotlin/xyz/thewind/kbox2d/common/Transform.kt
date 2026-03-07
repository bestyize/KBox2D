package xyz.thewind.kbox2d.common

class Transform {
    val p: Vec2
    val q: Rot

    constructor() {
        p = Vec2()
        q = Rot()
    }

    constructor(xf: Transform) {
        p = xf.p.clone()
        q = xf.q.clone()
    }

    constructor(position: Vec2, rotation: Rot) {
        p = position.clone()
        q = rotation.clone()
    }

    fun set(xf: Transform): Transform {
        p.set(xf.p)
        q.set(xf.q)
        return this
    }

    fun set(p: Vec2, angle: Float) {
        this.p.set(p)
        q.set(angle)
    }

    fun setIdentity() {
        p.setZero()
        q.setIdentity()
    }

    override fun toString(): String = buildString {
        append("XForm:\n")
        append("Position: ").append(p).append('\n')
        append("R: \n").append(q).append('\n')
    }

    companion object {
        private val pool = Vec2()

        fun mul(T: Transform, v: Vec2): Vec2 = Vec2(
            (T.q.c * v.x - T.q.s * v.y) + T.p.x,
            (T.q.s * v.x + T.q.c * v.y) + T.p.y,
        )

        fun mulToOut(T: Transform, v: Vec2, out: Vec2) {
            val tempY = (T.q.s * v.x + T.q.c * v.y) + T.p.y
            out.x = (T.q.c * v.x - T.q.s * v.y) + T.p.x
            out.y = tempY
        }

        fun mulToOutUnsafe(T: Transform, v: Vec2, out: Vec2) {
            assert(v !== out)
            out.x = (T.q.c * v.x - T.q.s * v.y) + T.p.x
            out.y = (T.q.s * v.x + T.q.c * v.y) + T.p.y
        }

        fun mulTrans(T: Transform, v: Vec2): Vec2 {
            val px = v.x - T.p.x
            val py = v.y - T.p.y
            return Vec2(T.q.c * px + T.q.s * py, -T.q.s * px + T.q.c * py)
        }

        fun mulTransToOut(T: Transform, v: Vec2, out: Vec2) {
            val px = v.x - T.p.x
            val py = v.y - T.p.y
            val tempY = -T.q.s * px + T.q.c * py
            out.x = T.q.c * px + T.q.s * py
            out.y = tempY
        }

        fun mulTransToOutUnsafe(T: Transform, v: Vec2, out: Vec2) {
            assert(v !== out)
            val px = v.x - T.p.x
            val py = v.y - T.p.y
            out.x = T.q.c * px + T.q.s * py
            out.y = -T.q.s * px + T.q.c * py
        }

        fun mul(A: Transform, B: Transform): Transform = Transform().also { C ->
            Rot.mulUnsafe(A.q, B.q, C.q)
            Rot.mulToOutUnsafe(A.q, B.p, C.p)
            C.p.addLocal(A.p)
        }

        fun mulToOut(A: Transform, B: Transform, out: Transform) {
            assert(out !== A)
            Rot.mul(A.q, B.q, out.q)
            Rot.mulToOut(A.q, B.p, out.p)
            out.p.addLocal(A.p)
        }

        fun mulToOutUnsafe(A: Transform, B: Transform, out: Transform) {
            assert(out !== A)
            assert(out !== B)
            Rot.mulUnsafe(A.q, B.q, out.q)
            Rot.mulToOutUnsafe(A.q, B.p, out.p)
            out.p.addLocal(A.p)
        }

        fun mulTrans(A: Transform, B: Transform): Transform = Transform().also { C ->
            Rot.mulTransUnsafe(A.q, B.q, C.q)
            pool.set(B.p).subLocal(A.p)
            Rot.mulTransUnsafe(A.q, pool, C.p)
        }

        fun mulTransToOut(A: Transform, B: Transform, out: Transform) {
            assert(out !== A)
            Rot.mulTrans(A.q, B.q, out.q)
            pool.set(B.p).subLocal(A.p)
            Rot.mulTrans(A.q, pool, out.p)
        }

        fun mulTransToOutUnsafe(A: Transform, B: Transform, out: Transform) {
            assert(out !== A)
            assert(out !== B)
            Rot.mulTransUnsafe(A.q, B.q, out.q)
            pool.set(B.p).subLocal(A.p)
            Rot.mulTransUnsafe(A.q, pool, out.p)
        }
    }
}
