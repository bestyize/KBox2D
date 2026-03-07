package xyz.thewind.kbox2d.particle

object ParticleType {
    const val b2_waterParticle: Int = 0
    const val b2_zombieParticle: Int = 1 shl 1
    const val b2_wallParticle: Int = 1 shl 2
    const val b2_springParticle: Int = 1 shl 3
    const val b2_elasticParticle: Int = 1 shl 4
    const val b2_viscousParticle: Int = 1 shl 5
    const val b2_powderParticle: Int = 1 shl 6
    const val b2_tensileParticle: Int = 1 shl 7
    const val b2_colorMixingParticle: Int = 1 shl 8
    const val b2_destructionListener: Int = 1 shl 9
}
