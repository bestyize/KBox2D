package xyz.thewind.kbox2d.particle

import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body

class ParticleBodyContact {
    var index: Int = 0
    var body: Body? = null
    var weight: Float = 0f
    val normal: Vec2 = Vec2()
    var mass: Float = 0f
}
