package xyz.thewind.kbox2d.collision

import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2

class AABB {
    val lowerBound: Vec2
    val upperBound: Vec2

    constructor() {
        lowerBound = Vec2()
        upperBound = Vec2()
    }

    constructor(copy: AABB) : this(copy.lowerBound, copy.upperBound)

    constructor(lowerVertex: Vec2, upperVertex: Vec2) {
        lowerBound = lowerVertex.clone()
        upperBound = upperVertex.clone()
    }

    fun set(aabb: AABB) {
        lowerBound.set(aabb.lowerBound)
        upperBound.set(aabb.upperBound)
    }

    fun isValid(): Boolean {
        val dx = upperBound.x - lowerBound.x
        if (dx < 0f) return false
        val dy = upperBound.y - lowerBound.y
        if (dy < 0f) return false
        return lowerBound.isValid() && upperBound.isValid()
    }

    fun getCenter(): Vec2 = Vec2(lowerBound).addLocal(upperBound).mulLocal(.5f)

    fun getCenterToOut(out: Vec2) {
        out.x = (lowerBound.x + upperBound.x) * .5f
        out.y = (lowerBound.y + upperBound.y) * .5f
    }

    fun getExtents(): Vec2 = Vec2(upperBound).subLocal(lowerBound).mulLocal(.5f)

    fun getExtentsToOut(out: Vec2) {
        out.x = (upperBound.x - lowerBound.x) * .5f
        out.y = (upperBound.y - lowerBound.y) * .5f
    }

    fun getVertices(ray: Array<Vec2>) {
        ray[0].set(lowerBound)
        ray[1].set(lowerBound)
        ray[1].x += upperBound.x - lowerBound.x
        ray[2].set(upperBound)
        ray[3].set(upperBound)
        ray[3].x -= upperBound.x - lowerBound.x
    }

    fun combine(aabb1: AABB, aabb2: AABB) {
        lowerBound.x = if (aabb1.lowerBound.x < aabb2.lowerBound.x) aabb1.lowerBound.x else aabb2.lowerBound.x
        lowerBound.y = if (aabb1.lowerBound.y < aabb2.lowerBound.y) aabb1.lowerBound.y else aabb2.lowerBound.y
        upperBound.x = if (aabb1.upperBound.x > aabb2.upperBound.x) aabb1.upperBound.x else aabb2.upperBound.x
        upperBound.y = if (aabb1.upperBound.y > aabb2.upperBound.y) aabb1.upperBound.y else aabb2.upperBound.y
    }

    fun getPerimeter(): Float = 2.0f * (upperBound.x - lowerBound.x + upperBound.y - lowerBound.y)

    fun combine(aabb: AABB) {
        lowerBound.x = if (lowerBound.x < aabb.lowerBound.x) lowerBound.x else aabb.lowerBound.x
        lowerBound.y = if (lowerBound.y < aabb.lowerBound.y) lowerBound.y else aabb.lowerBound.y
        upperBound.x = if (upperBound.x > aabb.upperBound.x) upperBound.x else aabb.upperBound.x
        upperBound.y = if (upperBound.y > aabb.upperBound.y) upperBound.y else aabb.upperBound.y
    }

    fun contains(aabb: AABB): Boolean =
        lowerBound.x <= aabb.lowerBound.x &&
            lowerBound.y <= aabb.lowerBound.y &&
            aabb.upperBound.x <= upperBound.x &&
            aabb.upperBound.y <= upperBound.y

    fun raycast(output: RayCastOutput, input: RayCastInput): Boolean {
        var tmin = -Float.MAX_VALUE
        var tmax = Float.MAX_VALUE

        val p = Vec2(input.p1)
        val d = Vec2(input.p2).subLocal(input.p1)
        val absD = Vec2.abs(d)
        val normal = Vec2()

        if (absD.x < Settings.EPSILON) {
            if (p.x < lowerBound.x || upperBound.x < p.x) return false
        } else {
            val invD = 1.0f / d.x
            var t1 = (lowerBound.x - p.x) * invD
            var t2 = (upperBound.x - p.x) * invD
            var s = -1.0f
            if (t1 > t2) {
                val temp = t1
                t1 = t2
                t2 = temp
                s = 1.0f
            }
            if (t1 > tmin) {
                normal.setZero()
                normal.x = s
                tmin = t1
            }
            tmax = MathUtils.min(tmax, t2)
            if (tmin > tmax) return false
        }

        if (absD.y < Settings.EPSILON) {
            if (p.y < lowerBound.y || upperBound.y < p.y) return false
        } else {
            val invD = 1.0f / d.y
            var t1 = (lowerBound.y - p.y) * invD
            var t2 = (upperBound.y - p.y) * invD
            var s = -1.0f
            if (t1 > t2) {
                val temp = t1
                t1 = t2
                t2 = temp
                s = 1.0f
            }
            if (t1 > tmin) {
                normal.setZero()
                normal.y = s
                tmin = t1
            }
            tmax = MathUtils.min(tmax, t2)
            if (tmin > tmax) return false
        }

        if (tmin < 0.0f || input.maxFraction < tmin) return false

        output.fraction = tmin
        output.normal.set(normal)
        return true
    }

    companion object {
        fun testOverlap(a: AABB, b: AABB): Boolean {
            if (b.lowerBound.x - a.upperBound.x > 0.0f || b.lowerBound.y - a.upperBound.y > 0.0f) {
                return false
            }
            if (a.lowerBound.x - b.upperBound.x > 0.0f || a.lowerBound.y - b.upperBound.y > 0.0f) {
                return false
            }
            return true
        }
    }
}
