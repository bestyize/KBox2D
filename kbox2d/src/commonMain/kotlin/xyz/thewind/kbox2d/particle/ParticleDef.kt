package xyz.thewind.kbox2d.particle

import xyz.thewind.kbox2d.common.Vec2

class ParticleDef {
    var flags: Int = 0
    val position: Vec2 = Vec2()
    val velocity: Vec2 = Vec2()
    var color: ParticleColor? = null
    var userData: Any? = null
}
