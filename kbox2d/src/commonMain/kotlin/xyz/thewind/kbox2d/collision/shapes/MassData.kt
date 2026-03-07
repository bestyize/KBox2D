package xyz.thewind.kbox2d.collision.shapes

import xyz.thewind.kbox2d.common.Vec2

class MassData() {
    var mass: Float = 0f
    val center: Vec2 = Vec2()
    var I: Float = 0f

    constructor(md: MassData) : this() {
        mass = md.mass
        I = md.I
        center.set(md.center)
    }

    fun set(md: MassData) {
        mass = md.mass
        I = md.I
        center.set(md.center)
    }

    fun clone(): MassData = MassData(this)
}
