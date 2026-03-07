package xyz.thewind.kbox2d.callbacks

import xyz.thewind.kbox2d.particle.ParticleGroup

interface ParticleDestructionListener {
    fun sayGoodbye(group: ParticleGroup)
    fun sayGoodbye(index: Int)
}
