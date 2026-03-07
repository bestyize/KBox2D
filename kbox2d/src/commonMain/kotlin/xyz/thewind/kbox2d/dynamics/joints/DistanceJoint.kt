package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.pooling.IWorldPool

class DistanceJoint(
    pool: IWorldPool,
    def: DistanceJointDef,
) : Joint(pool, def) {
    private var m_frequencyHz: Float = def.frequencyHz
    private var m_dampingRatio: Float = def.dampingRatio
    private var m_bias: Float = 0f
    private val m_localAnchorA: Vec2 = def.localAnchorA.clone()
    private val m_localAnchorB: Vec2 = def.localAnchorB.clone()
    private var m_gamma: Float = 0f
    private var m_impulse: Float = 0f
    private var m_length: Float = def.length
    private var m_indexA: Int = 0
    private var m_indexB: Int = 0
    private val m_u: Vec2 = Vec2()
    private val m_rA: Vec2 = Vec2()
    private val m_rB: Vec2 = Vec2()
    private val m_localCenterA: Vec2 = Vec2()
    private val m_localCenterB: Vec2 = Vec2()
    private var m_invMassA: Float = 0f
    private var m_invMassB: Float = 0f
    private var m_invIA: Float = 0f
    private var m_invIB: Float = 0f
    private var m_mass: Float = 0f

    fun setFrequency(hz: Float) { m_frequencyHz = hz }
    fun getFrequency(): Float = m_frequencyHz
    fun getLength(): Float = m_length
    fun setLength(length: Float) { m_length = length }
    fun setDampingRatio(damp: Float) { m_dampingRatio = damp }
    fun getDampingRatio(): Float = m_dampingRatio
    fun getLocalAnchorA(): Vec2 = m_localAnchorA
    fun getLocalAnchorB(): Vec2 = m_localAnchorB

    override fun getAnchorA(out: Vec2) {
        getBodyA().getWorldPointToOut(m_localAnchorA, out)
    }

    override fun getAnchorB(out: Vec2) {
        getBodyB().getWorldPointToOut(m_localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.x = m_impulse * m_u.x * invDt
        out.y = m_impulse * m_u.y * invDt
    }

    override fun getReactionTorque(invDt: Float): Float = 0f

    override fun initVelocityConstraints(data: SolverData) {
        val step = data.step!!
        m_indexA = getBodyA().m_islandIndex
        m_indexB = getBodyB().m_islandIndex
        m_localCenterA.set(getBodyA().m_sweep.localCenter)
        m_localCenterB.set(getBodyB().m_sweep.localCenter)
        m_invMassA = getBodyA().m_invMass
        m_invMassB = getBodyB().m_invMass
        m_invIA = getBodyA().m_invI
        m_invIB = getBodyB().m_invI

        val cA = data.positions[m_indexA].c
        val aA = data.positions[m_indexA].a
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val cB = data.positions[m_indexB].c
        val aB = data.positions[m_indexB].a
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w

        val qA = pool.popRot()
        val qB = pool.popRot()
        qA.set(aA)
        qB.set(aB)

        Rot.mulToOutUnsafe(qA, m_u.set(m_localAnchorA).subLocal(m_localCenterA), m_rA)
        Rot.mulToOutUnsafe(qB, m_u.set(m_localAnchorB).subLocal(m_localCenterB), m_rB)
        m_u.set(cB).addLocal(m_rB).subLocal(cA).subLocal(m_rA)
        pool.pushRot(2)

        val length = m_u.length()
        if (length > Settings.linearSlop) {
            m_u.mulLocal(1f / length)
        } else {
            m_u.setZero()
        }

        val crAu = Vec2.cross(m_rA, m_u)
        val crBu = Vec2.cross(m_rB, m_u)
        var invMass = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu
        m_mass = if (invMass != 0f) 1f / invMass else 0f

        if (m_frequencyHz > 0f) {
            val c = length - m_length
            val omega = 2f * MathUtils.PI * m_frequencyHz
            val d = 2f * m_mass * m_dampingRatio * omega
            val k = m_mass * omega * omega
            val h = step.dt
            m_gamma = h * (d + h * k)
            m_gamma = if (m_gamma != 0f) 1f / m_gamma else 0f
            m_bias = c * h * k * m_gamma
            invMass += m_gamma
            m_mass = if (invMass != 0f) 1f / invMass else 0f
        } else {
            m_gamma = 0f
            m_bias = 0f
        }

        if (step.warmStarting) {
            m_impulse *= step.dtRatio
            val p = pool.popVec2()
            p.set(m_u).mulLocal(m_impulse)
            vA.x -= m_invMassA * p.x
            vA.y -= m_invMassA * p.y
            wA -= m_invIA * Vec2.cross(m_rA, p)
            vB.x += m_invMassB * p.x
            vB.y += m_invMassB * p.y
            wB += m_invIB * Vec2.cross(m_rB, p)
            pool.pushVec2(1)
        } else {
            m_impulse = 0f
        }
        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
    }

    override fun solveVelocityConstraints(data: SolverData) {
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w
        val vpA = pool.popVec2()
        val vpB = pool.popVec2()
        Vec2.crossToOutUnsafe(wA, m_rA, vpA)
        vpA.addLocal(vA)
        Vec2.crossToOutUnsafe(wB, m_rB, vpB)
        vpB.addLocal(vB)
        val cDot = Vec2.dot(m_u, vpB.subLocal(vpA))
        val impulse = -m_mass * (cDot + m_bias + m_gamma * m_impulse)
        m_impulse += impulse
        val px = impulse * m_u.x
        val py = impulse * m_u.y
        vA.x -= m_invMassA * px
        vA.y -= m_invMassA * py
        wA -= m_invIA * (m_rA.x * py - m_rA.y * px)
        vB.x += m_invMassB * px
        vB.y += m_invMassB * py
        wB += m_invIB * (m_rB.x * py - m_rB.y * px)
        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
        pool.pushVec2(2)
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        if (m_frequencyHz > 0f) {
            return true
        }
        val qA = pool.popRot()
        val qB = pool.popRot()
        val rA = pool.popVec2()
        val rB = pool.popVec2()
        val u = pool.popVec2()
        val cA = data.positions[m_indexA].c
        var aA = data.positions[m_indexA].a
        val cB = data.positions[m_indexB].c
        var aB = data.positions[m_indexB].a
        qA.set(aA)
        qB.set(aB)
        Rot.mulToOutUnsafe(qA, u.set(m_localAnchorA).subLocal(m_localCenterA), rA)
        Rot.mulToOutUnsafe(qB, u.set(m_localAnchorB).subLocal(m_localCenterB), rB)
        u.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)
        val length = u.normalize()
        var c = length - m_length
        c = MathUtils.clamp(c, -Settings.maxLinearCorrection, Settings.maxLinearCorrection)
        val impulse = -m_mass * c
        val px = impulse * u.x
        val py = impulse * u.y
        cA.x -= m_invMassA * px
        cA.y -= m_invMassA * py
        aA -= m_invIA * (rA.x * py - rA.y * px)
        cB.x += m_invMassB * px
        cB.y += m_invMassB * py
        aB += m_invIB * (rB.x * py - rB.y * px)
        data.positions[m_indexA].a = aA
        data.positions[m_indexB].a = aB
        pool.pushVec2(3)
        pool.pushRot(2)
        return MathUtils.abs(c) < Settings.linearSlop
    }
}
