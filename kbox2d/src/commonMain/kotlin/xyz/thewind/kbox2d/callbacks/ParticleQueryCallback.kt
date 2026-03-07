package xyz.thewind.kbox2d.callbacks

fun interface ParticleQueryCallback {
    fun reportParticle(index: Int): Boolean
}
