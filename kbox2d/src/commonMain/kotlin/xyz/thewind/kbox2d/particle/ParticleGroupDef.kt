package xyz.thewind.kbox2d.particle

import xyz.thewind.kbox2d.collision.shapes.Shape
import xyz.thewind.kbox2d.common.Vec2

class ParticleGroupDef {
    var flags: Int = 0
    var groupFlags: Int = 0
    val position: Vec2 = Vec2()
    var angle: Float = 0f
    val linearVelocity: Vec2 = Vec2()
    var angularVelocity: Float = 0f
    var color: ParticleColor? = null
    var strength: Float = 1f
    var shape: Shape? = null
    var destroyAutomatically: Boolean = true
    var userData: Any? = null
}
