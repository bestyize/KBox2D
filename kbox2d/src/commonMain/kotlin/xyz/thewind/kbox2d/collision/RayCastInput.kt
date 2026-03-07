package xyz.thewind.kbox2d.collision

import xyz.thewind.kbox2d.common.Vec2

class RayCastInput {
    val p1: Vec2 = Vec2()
    val p2: Vec2 = Vec2()
    var maxFraction: Float = 0f

    fun set(rci: RayCastInput) {
        p1.set(rci.p1)
        p2.set(rci.p2)
        maxFraction = rci.maxFraction
    }
}
