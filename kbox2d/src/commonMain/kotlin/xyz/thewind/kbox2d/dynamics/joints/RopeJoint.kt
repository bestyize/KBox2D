package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.pooling.IWorldPool

class RopeJoint(
    pool: IWorldPool,
    def: RopeJointDef,
) : Joint(pool, def) {
    private val m_localAnchorA: Vec2 = def.localAnchorA.clone()
    private val m_localAnchorB: Vec2 = def.localAnchorB.clone()
    private var m_maxLength: Float = def.maxLength
    private var m_length: Float = 0f
    private var m_impulse: Float = 0f
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
    private var m_state: LimitState = LimitState.INACTIVE

    fun getLocalAnchorA(): Vec2 = m_localAnchorA
    fun getLocalAnchorB(): Vec2 = m_localAnchorB
    fun getMaxLength(): Float = m_maxLength
    fun setMaxLength(maxLength: Float) { m_maxLength = maxLength }
    fun getLimitState(): LimitState = m_state

    override fun getAnchorA(out: Vec2) {
        getBodyA().getWorldPointToOut(m_localAnchorA, out)
    }

    override fun getAnchorB(out: Vec2) {
        getBodyB().getWorldPointToOut(m_localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.set(m_u).mulLocal(invDt).mulLocal(m_impulse)
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
        val temp = pool.popVec2()
        qA.set(aA)
        qB.set(aB)

        Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), m_rA)
        Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), m_rB)
        m_u.set(cB).addLocal(m_rB).subLocal(cA).subLocal(m_rA)
        m_length = m_u.length()
        val c = m_length - m_maxLength
        m_state = if (c > 0f) LimitState.AT_UPPER else LimitState.INACTIVE

        if (m_length > Settings.linearSlop) {
            m_u.mulLocal(1f / m_length)
        } else {
            m_u.setZero()
            m_mass = 0f
            m_impulse = 0f
            pool.pushRot(2)
            pool.pushVec2(1)
            return
        }

        val crA = Vec2.cross(m_rA, m_u)
        val crB = Vec2.cross(m_rB, m_u)
        val invMass = m_invMassA + m_invIA * crA * crA + m_invMassB + m_invIB * crB * crB
        m_mass = if (invMass != 0f) 1f / invMass else 0f

        if (step.warmStarting) {
            m_impulse *= step.dtRatio
            val px = m_impulse * m_u.x
            val py = m_impulse * m_u.y
            vA.x -= m_invMassA * px
            vA.y -= m_invMassA * py
            wA -= m_invIA * (m_rA.x * py - m_rA.y * px)
            vB.x += m_invMassB * px
            vB.y += m_invMassB * py
            wB += m_invIB * (m_rB.x * py - m_rB.y * px)
        } else {
            m_impulse = 0f
        }

        pool.pushRot(2)
        pool.pushVec2(1)
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
        val temp = pool.popVec2()
        Vec2.crossToOutUnsafe(wA, m_rA, vpA)
        vpA.addLocal(vA)
        Vec2.crossToOutUnsafe(wB, m_rB, vpB)
        vpB.addLocal(vB)
        val c = m_length - m_maxLength
        var cDot = Vec2.dot(m_u, temp.set(vpB).subLocal(vpA))
        if (c < 0f) {
            cDot += data.step!!.inv_dt * c
        }
        var impulse = -m_mass * cDot
        val oldImpulse = m_impulse
        m_impulse = MathUtils.min(0f, m_impulse + impulse)
        impulse = m_impulse - oldImpulse
        val px = impulse * m_u.x
        val py = impulse * m_u.y
        vA.x -= m_invMassA * px
        vA.y -= m_invMassA * py
        wA -= m_invIA * (m_rA.x * py - m_rA.y * px)
        vB.x += m_invMassB * px
        vB.y += m_invMassB * py
        wB += m_invIB * (m_rB.x * py - m_rB.y * px)
        pool.pushVec2(3)
        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        val cA = data.positions[m_indexA].c
        var aA = data.positions[m_indexA].a
        val cB = data.positions[m_indexB].c
        var aB = data.positions[m_indexB].a
        val qA = pool.popRot()
        val qB = pool.popRot()
        val u = pool.popVec2()
        val rA = pool.popVec2()
        val rB = pool.popVec2()
        val temp = pool.popVec2()
        qA.set(aA)
        qB.set(aB)
        Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), rA)
        Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), rB)
        u.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)
        val length = u.normalize()
        var c = length - m_maxLength
        c = MathUtils.clamp(c, 0f, Settings.maxLinearCorrection)
        val impulse = -m_mass * c
        val px = impulse * u.x
        val py = impulse * u.y
        cA.x -= m_invMassA * px
        cA.y -= m_invMassA * py
        aA -= m_invIA * (rA.x * py - rA.y * px)
        cB.x += m_invMassB * px
        cB.y += m_invMassB * py
        aB += m_invIB * (rB.x * py - rB.y * px)
        pool.pushRot(2)
        pool.pushVec2(4)
        data.positions[m_indexA].a = aA
        data.positions[m_indexB].a = aB
        return length - m_maxLength < Settings.linearSlop
    }
}
