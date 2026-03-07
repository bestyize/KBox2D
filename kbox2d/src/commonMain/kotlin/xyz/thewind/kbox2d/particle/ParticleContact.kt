package xyz.thewind.kbox2d.particle

import xyz.thewind.kbox2d.common.Vec2

class ParticleContact {
    var indexA: Int = 0
    var indexB: Int = 0
    var flags: Int = 0
    var weight: Float = 0f
    val normal: Vec2 = Vec2()
}
