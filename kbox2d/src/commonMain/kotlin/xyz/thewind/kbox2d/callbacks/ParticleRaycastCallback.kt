package xyz.thewind.kbox2d.callbacks

import xyz.thewind.kbox2d.common.Vec2

fun interface ParticleRaycastCallback {
    fun reportParticle(index: Int, point: Vec2, normal: Vec2, fraction: Float): Float
}
