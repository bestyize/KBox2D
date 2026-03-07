package xyz.thewind.kbox2d.collision

import xyz.thewind.kbox2d.common.Vec2

class RayCastOutput {
    val normal: Vec2 = Vec2()
    var fraction: Float = 0f

    fun set(rco: RayCastOutput) {
        normal.set(rco.normal)
        fraction = rco.fraction
    }
}
