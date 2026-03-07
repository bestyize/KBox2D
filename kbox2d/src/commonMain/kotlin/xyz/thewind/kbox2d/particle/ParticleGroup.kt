package xyz.thewind.kbox2d.particle

import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2

open class ParticleGroup {
    internal var m_system: ParticleSystem? = null
    internal var m_firstIndex: Int = 0
    internal var m_lastIndex: Int = 0
    internal var m_groupFlags: Int = 0
    internal var m_strength: Float = 1f
    internal var m_prev: ParticleGroup? = null
    internal var m_next: ParticleGroup? = null
    internal var m_timestamp: Int = -1
    internal var m_mass: Float = 0f
    internal var m_inertia: Float = 0f
    internal val m_center: Vec2 = Vec2()
    internal val m_linearVelocity: Vec2 = Vec2()
    internal var m_angularVelocity: Float = 0f
    internal val m_transform: Transform = Transform()
    internal var m_destroyAutomatically: Boolean = true
    internal var m_toBeDestroyed: Boolean = false
    internal var m_toBeSplit: Boolean = false
    internal var m_userData: Any? = null
    internal val m_localPositions: MutableList<Vec2> = mutableListOf()

    fun getNext(): ParticleGroup? = m_next
    fun getParticleCount(): Int = m_lastIndex - m_firstIndex
    fun getBufferIndex(): Int = m_firstIndex
    fun getGroupFlags(): Int = m_groupFlags
    fun setGroupFlags(flags: Int) { m_groupFlags = flags }
    fun getMass(): Float { updateStatistics(); return m_mass }
    fun getInertia(): Float { updateStatistics(); return m_inertia }
    fun getCenter(): Vec2 { updateStatistics(); return m_center }
    fun getLinearVelocity(): Vec2 { updateStatistics(); return m_linearVelocity }
    fun getAngularVelocity(): Float { updateStatistics(); return m_angularVelocity }
    fun getTransform(): Transform = m_transform
    fun getPosition(): Vec2 = m_transform.p
    fun getAngle(): Float = m_transform.q.getAngle()
    fun getUserData(): Any? = m_userData
    fun setUserData(data: Any?) { m_userData = data }

    fun updateStatistics() {
        val system = m_system ?: return
        if (m_timestamp == system.m_timestamp) {
            return
        }
        val mass = system.getParticleMass()
        m_mass = 0f
        m_center.setZero()
        m_linearVelocity.setZero()
        for (i in m_firstIndex until m_lastIndex) {
            m_mass += mass
            val pos = system.positionBuffer[i]
            m_center.x += mass * pos.x
            m_center.y += mass * pos.y
            val vel = system.velocityBuffer[i]
            m_linearVelocity.x += mass * vel.x
            m_linearVelocity.y += mass * vel.y
        }
        if (m_mass > 0f) {
            m_center.mulLocal(1f / m_mass)
            m_linearVelocity.mulLocal(1f / m_mass)
        }
        m_inertia = 0f
        m_angularVelocity = 0f
        for (i in m_firstIndex until m_lastIndex) {
            val pos = system.positionBuffer[i]
            val vel = system.velocityBuffer[i]
            val px = pos.x - m_center.x
            val py = pos.y - m_center.y
            val vx = vel.x - m_linearVelocity.x
            val vy = vel.y - m_linearVelocity.y
            m_inertia += mass * (px * px + py * py)
            m_angularVelocity += mass * (px * vy - py * vx)
        }
        if (m_inertia > 0f) {
            m_angularVelocity *= 1f / m_inertia
        }
        m_timestamp = system.m_timestamp
    }
}
