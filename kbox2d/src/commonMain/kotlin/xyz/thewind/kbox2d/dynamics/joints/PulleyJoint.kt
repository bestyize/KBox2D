package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.pooling.IWorldPool

class PulleyJoint(
    pool: IWorldPool,
    def: PulleyJointDef,
) : Joint(pool, def) {
    companion object {
        const val MIN_PULLEY_LENGTH: Float = 2f
    }

    private val m_groundAnchorA: Vec2 = def.groundAnchorA.clone()
    private val m_groundAnchorB: Vec2 = def.groundAnchorB.clone()
    private var m_lengthA: Float = def.lengthA
    private var m_lengthB: Float = def.lengthB
    private val m_localAnchorA: Vec2 = def.localAnchorA.clone()
    private val m_localAnchorB: Vec2 = def.localAnchorB.clone()
    private var m_constant: Float = def.lengthA + def.ratio * def.lengthB
    private var m_ratio: Float = def.ratio
    private var m_impulse: Float = 0f
    private var m_indexA: Int = 0
    private var m_indexB: Int = 0
    private val m_uA: Vec2 = Vec2()
    private val m_uB: Vec2 = Vec2()
    private val m_rA: Vec2 = Vec2()
    private val m_rB: Vec2 = Vec2()
    private val m_localCenterA: Vec2 = Vec2()
    private val m_localCenterB: Vec2 = Vec2()
    private var m_invMassA: Float = 0f
    private var m_invMassB: Float = 0f
    private var m_invIA: Float = 0f
    private var m_invIB: Float = 0f
    private var m_mass: Float = 0f

    fun getLengthA(): Float = m_lengthA
    fun getLengthB(): Float = m_lengthB
    fun getLocalAnchorA(): Vec2 = m_localAnchorA
    fun getLocalAnchorB(): Vec2 = m_localAnchorB
    fun getGroundAnchorA(): Vec2 = m_groundAnchorA
    fun getGroundAnchorB(): Vec2 = m_groundAnchorB
    fun getRatio(): Float = m_ratio

    fun getCurrentLengthA(): Float {
        val p = pool.popVec2()
        m_bodyA.getWorldPointToOut(m_localAnchorA, p)
        p.subLocal(m_groundAnchorA)
        val length = p.length()
        pool.pushVec2(1)
        return length
    }

    fun getCurrentLengthB(): Float {
        val p = pool.popVec2()
        m_bodyB.getWorldPointToOut(m_localAnchorB, p)
        p.subLocal(m_groundAnchorB)
        val length = p.length()
        pool.pushVec2(1)
        return length
    }

    fun getLength1(): Float = getCurrentLengthA()

    fun getLength2(): Float = getCurrentLengthB()

    override fun getAnchorA(out: Vec2) {
        m_bodyA.getWorldPointToOut(m_localAnchorA, out)
    }

    override fun getAnchorB(out: Vec2) {
        m_bodyB.getWorldPointToOut(m_localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.set(m_uB).mulLocal(m_impulse).mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float = 0f

    override fun initVelocityConstraints(data: SolverData) {
        val step = data.step!!
        m_indexA = m_bodyA.m_islandIndex
        m_indexB = m_bodyB.m_islandIndex
        m_localCenterA.set(m_bodyA.m_sweep.localCenter)
        m_localCenterB.set(m_bodyB.m_sweep.localCenter)
        m_invMassA = m_bodyA.m_invMass
        m_invMassB = m_bodyB.m_invMass
        m_invIA = m_bodyA.m_invI
        m_invIB = m_bodyB.m_invI

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
        m_uA.set(cA).addLocal(m_rA).subLocal(m_groundAnchorA)
        m_uB.set(cB).addLocal(m_rB).subLocal(m_groundAnchorB)

        val lengthA = m_uA.length()
        val lengthB = m_uB.length()
        if (lengthA > 10f * Settings.linearSlop) {
            m_uA.mulLocal(1f / lengthA)
        } else {
            m_uA.setZero()
        }
        if (lengthB > 10f * Settings.linearSlop) {
            m_uB.mulLocal(1f / lengthB)
        } else {
            m_uB.setZero()
        }

        val ruA = Vec2.cross(m_rA, m_uA)
        val ruB = Vec2.cross(m_rB, m_uB)
        val massA = m_invMassA + m_invIA * ruA * ruA
        val massB = m_invMassB + m_invIB * ruB * ruB
        m_mass = massA + m_ratio * m_ratio * massB
        if (m_mass > 0f) {
            m_mass = 1f / m_mass
        }

        if (step.warmStarting) {
            m_impulse *= step.dtRatio
            val pA = pool.popVec2()
            val pB = pool.popVec2()
            pA.set(m_uA).mulLocal(-m_impulse)
            pB.set(m_uB).mulLocal(-m_ratio * m_impulse)
            vA.x += m_invMassA * pA.x
            vA.y += m_invMassA * pA.y
            wA += m_invIA * Vec2.cross(m_rA, pA)
            vB.x += m_invMassB * pB.x
            vB.y += m_invMassB * pB.y
            wB += m_invIB * Vec2.cross(m_rB, pB)
            pool.pushVec2(2)
        } else {
            m_impulse = 0f
        }

        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
        pool.pushVec2(1)
        pool.pushRot(2)
    }

    override fun solveVelocityConstraints(data: SolverData) {
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w

        val vpA = pool.popVec2()
        val vpB = pool.popVec2()
        val pA = pool.popVec2()
        val pB = pool.popVec2()
        Vec2.crossToOutUnsafe(wA, m_rA, vpA)
        vpA.addLocal(vA)
        Vec2.crossToOutUnsafe(wB, m_rB, vpB)
        vpB.addLocal(vB)

        val cDot = -Vec2.dot(m_uA, vpA) - m_ratio * Vec2.dot(m_uB, vpB)
        val impulse = -m_mass * cDot
        m_impulse += impulse

        pA.set(m_uA).mulLocal(-impulse)
        pB.set(m_uB).mulLocal(-m_ratio * impulse)
        vA.x += m_invMassA * pA.x
        vA.y += m_invMassA * pA.y
        wA += m_invIA * Vec2.cross(m_rA, pA)
        vB.x += m_invMassB * pB.x
        vB.y += m_invMassB * pB.y
        wB += m_invIB * Vec2.cross(m_rB, pB)

        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
        pool.pushVec2(4)
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        val qA = pool.popRot()
        val qB = pool.popRot()
        val rA = pool.popVec2()
        val rB = pool.popVec2()
        val uA = pool.popVec2()
        val uB = pool.popVec2()
        val temp = pool.popVec2()
        val pA = pool.popVec2()
        val pB = pool.popVec2()

        val cA = data.positions[m_indexA].c
        var aA = data.positions[m_indexA].a
        val cB = data.positions[m_indexB].c
        var aB = data.positions[m_indexB].a

        qA.set(aA)
        qB.set(aB)
        Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), rA)
        Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), rB)
        uA.set(cA).addLocal(rA).subLocal(m_groundAnchorA)
        uB.set(cB).addLocal(rB).subLocal(m_groundAnchorB)

        val lengthA = uA.length()
        val lengthB = uB.length()
        if (lengthA > 10f * Settings.linearSlop) {
            uA.mulLocal(1f / lengthA)
        } else {
            uA.setZero()
        }
        if (lengthB > 10f * Settings.linearSlop) {
            uB.mulLocal(1f / lengthB)
        } else {
            uB.setZero()
        }

        val ruA = Vec2.cross(rA, uA)
        val ruB = Vec2.cross(rB, uB)
        val massA = m_invMassA + m_invIA * ruA * ruA
        val massB = m_invMassB + m_invIB * ruB * ruB
        var mass = massA + m_ratio * m_ratio * massB
        if (mass > 0f) {
            mass = 1f / mass
        }

        val c = m_constant - lengthA - m_ratio * lengthB
        val linearError = MathUtils.abs(c)
        val impulse = -mass * c
        pA.set(uA).mulLocal(-impulse)
        pB.set(uB).mulLocal(-m_ratio * impulse)

        cA.x += m_invMassA * pA.x
        cA.y += m_invMassA * pA.y
        aA += m_invIA * Vec2.cross(rA, pA)
        cB.x += m_invMassB * pB.x
        cB.y += m_invMassB * pB.y
        aB += m_invIB * Vec2.cross(rB, pB)

        data.positions[m_indexA].a = aA
        data.positions[m_indexB].a = aB
        pool.pushRot(2)
        pool.pushVec2(7)
        return linearError < Settings.linearSlop
    }
}
