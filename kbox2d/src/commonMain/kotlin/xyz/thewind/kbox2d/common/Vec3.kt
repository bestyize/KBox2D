package xyz.thewind.kbox2d.common

class Vec3(var x: Float = 0f, var y: Float = 0f, var z: Float = 0f) {
    constructor(copy: Vec3) : this(copy.x, copy.y, copy.z)

    fun set(vec: Vec3): Vec3 {
        x = vec.x
        y = vec.y
        z = vec.z
        return this
    }

    fun set(argX: Float, argY: Float, argZ: Float): Vec3 {
        x = argX
        y = argY
        z = argZ
        return this
    }

    fun addLocal(argVec: Vec3): Vec3 {
        x += argVec.x
        y += argVec.y
        z += argVec.z
        return this
    }

    fun add(argVec: Vec3): Vec3 = Vec3(x + argVec.x, y + argVec.y, z + argVec.z)

    fun subLocal(argVec: Vec3): Vec3 {
        x -= argVec.x
        y -= argVec.y
        z -= argVec.z
        return this
    }

    fun sub(argVec: Vec3): Vec3 = Vec3(x - argVec.x, y - argVec.y, z - argVec.z)

    fun mulLocal(argScalar: Float): Vec3 {
        x *= argScalar
        y *= argScalar
        z *= argScalar
        return this
    }

    fun mul(argScalar: Float): Vec3 = Vec3(x * argScalar, y * argScalar, z * argScalar)

    fun negate(): Vec3 = Vec3(-x, -y, -z)

    fun negateLocal(): Vec3 {
        x = -x
        y = -y
        z = -z
        return this
    }

    fun setZero() {
        x = 0f
        y = 0f
        z = 0f
    }

    fun clone(): Vec3 = Vec3(this)

    override fun toString(): String = "($x,$y,$z)"

    override fun hashCode(): Int {
        var result = 1
        result = 31 * result + x.toBits()
        result = 31 * result + y.toBits()
        result = 31 * result + z.toBits()
        return result
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Vec3) return false
        return x.toBits() == other.x.toBits() && y.toBits() == other.y.toBits() && z.toBits() == other.z.toBits()
    }

    companion object {
        fun dot(a: Vec3, b: Vec3): Float = a.x * b.x + a.y * b.y + a.z * b.z

        fun cross(a: Vec3, b: Vec3): Vec3 = Vec3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x,
        )

        fun crossToOut(a: Vec3, b: Vec3, out: Vec3) {
            val tempY = a.z * b.x - a.x * b.z
            val tempZ = a.x * b.y - a.y * b.x
            out.x = a.y * b.z - a.z * b.y
            out.y = tempY
            out.z = tempZ
        }

        fun crossToOutUnsafe(a: Vec3, b: Vec3, out: Vec3) {
            assert(out !== b)
            assert(out !== a)
            out.x = a.y * b.z - a.z * b.y
            out.y = a.z * b.x - a.x * b.z
            out.z = a.x * b.y - a.y * b.x
        }
    }
}
