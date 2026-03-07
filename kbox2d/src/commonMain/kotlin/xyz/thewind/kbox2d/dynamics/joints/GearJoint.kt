package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.pooling.IWorldPool

class GearJoint(
    pool: IWorldPool,
    def: GearJointDef,
) : Joint(pool, def) {
    private val m_joint1: Joint = requireNotNull(def.joint1)
    private val m_joint2: Joint = requireNotNull(def.joint2)
    private val m_typeA: JointType = m_joint1.getType()
    private val m_typeB: JointType = m_joint2.getType()
    private val m_bodyC: Body = m_joint1.getBodyA()
    private val m_bodyD: Body = m_joint2.getBodyA()
    private val m_localAnchorA: Vec2 = Vec2()
    private val m_localAnchorB: Vec2 = Vec2()
    private val m_localAnchorC: Vec2 = Vec2()
    private val m_localAnchorD: Vec2 = Vec2()
    private val m_localAxisC: Vec2 = Vec2()
    private val m_localAxisD: Vec2 = Vec2()
    private var m_referenceAngleA: Float = 0f
    private var m_referenceAngleB: Float = 0f
    private var m_constant: Float = 0f
    private var m_ratio: Float = def.ratio
    private var m_impulse: Float = 0f
    private var m_indexA: Int = 0
    private var m_indexB: Int = 0
    private var m_indexC: Int = 0
    private var m_indexD: Int = 0
    private val m_lcA: Vec2 = Vec2()
    private val m_lcB: Vec2 = Vec2()
    private val m_lcC: Vec2 = Vec2()
    private val m_lcD: Vec2 = Vec2()
    private var m_mA: Float = 0f
    private var m_mB: Float = 0f
    private var m_mC: Float = 0f
    private var m_mD: Float = 0f
    private var m_iA: Float = 0f
    private var m_iB: Float = 0f
    private var m_iC: Float = 0f
    private var m_iD: Float = 0f
    private val m_JvAC: Vec2 = Vec2()
    private val m_JvBD: Vec2 = Vec2()
    private var m_JwA: Float = 0f
    private var m_JwB: Float = 0f
    private var m_JwC: Float = 0f
    private var m_JwD: Float = 0f
    private var m_mass: Float = 0f

    init {
        require(m_typeA == JointType.REVOLUTE || m_typeA == JointType.PRISMATIC)
        require(m_typeB == JointType.REVOLUTE || m_typeB == JointType.PRISMATIC)

        val coordinateA: Float
        val coordinateB: Float
        if (m_typeA == JointType.REVOLUTE) {
            val revolute = m_joint1 as RevoluteJoint
            m_localAnchorC.set(revolute.getLocalAnchorA())
            m_localAnchorA.set(revolute.getLocalAnchorB())
            m_referenceAngleA = revolute.getReferenceAngle()
            m_localAxisC.setZero()
            coordinateA = m_bodyA.m_sweep.a - m_bodyC.m_sweep.a - m_referenceAngleA
        } else {
            val prismatic = m_joint1 as PrismaticJoint
            val pA = Vec2()
            val temp = Vec2()
            m_localAnchorC.set(prismatic.getLocalAnchorA())
            m_localAnchorA.set(prismatic.getLocalAnchorB())
            m_referenceAngleA = prismatic.getReferenceAngle()
            m_localAxisC.set(prismatic.getLocalAxisA())
            Rot.mulToOutUnsafe(m_bodyA.m_xf.q, m_localAnchorA, temp)
            temp.addLocal(m_bodyA.m_xf.p).subLocal(m_bodyC.m_xf.p)
            Rot.mulTransUnsafe(m_bodyC.m_xf.q, temp, pA)
            coordinateA = Vec2.dot(pA.subLocal(m_localAnchorC), m_localAxisC)
        }

        if (m_typeB == JointType.REVOLUTE) {
            val revolute = m_joint2 as RevoluteJoint
            m_localAnchorD.set(revolute.getLocalAnchorA())
            m_localAnchorB.set(revolute.getLocalAnchorB())
            m_referenceAngleB = revolute.getReferenceAngle()
            m_localAxisD.setZero()
            coordinateB = m_bodyB.m_sweep.a - m_bodyD.m_sweep.a - m_referenceAngleB
        } else {
            val prismatic = m_joint2 as PrismaticJoint
            val pB = Vec2()
            val temp = Vec2()
            m_localAnchorD.set(prismatic.getLocalAnchorA())
            m_localAnchorB.set(prismatic.getLocalAnchorB())
            m_referenceAngleB = prismatic.getReferenceAngle()
            m_localAxisD.set(prismatic.getLocalAxisA())
            Rot.mulToOutUnsafe(m_bodyB.m_xf.q, m_localAnchorB, temp)
            temp.addLocal(m_bodyB.m_xf.p).subLocal(m_bodyD.m_xf.p)
            Rot.mulTransUnsafe(m_bodyD.m_xf.q, temp, pB)
            coordinateB = Vec2.dot(pB.subLocal(m_localAnchorD), m_localAxisD)
        }

        m_constant = coordinateA + m_ratio * coordinateB
    }

    override fun getAnchorA(out: Vec2) {
        m_bodyA.getWorldPointToOut(m_localAnchorA, out)
    }

    override fun getAnchorB(out: Vec2) {
        m_bodyB.getWorldPointToOut(m_localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.set(m_JvAC).mulLocal(m_impulse * invDt)
    }

    override fun getReactionTorque(invDt: Float): Float = invDt * (m_impulse * m_JwA)

    fun setRatio(ratio: Float) {
        m_ratio = ratio
    }

    fun getRatio(): Float = m_ratio

    fun getJoint1(): Joint = m_joint1

    fun getJoint2(): Joint = m_joint2

    override fun initVelocityConstraints(data: SolverData) {
        val step = data.step!!
        m_indexA = m_bodyA.m_islandIndex
        m_indexB = m_bodyB.m_islandIndex
        m_indexC = m_bodyC.m_islandIndex
        m_indexD = m_bodyD.m_islandIndex
        m_lcA.set(m_bodyA.m_sweep.localCenter)
        m_lcB.set(m_bodyB.m_sweep.localCenter)
        m_lcC.set(m_bodyC.m_sweep.localCenter)
        m_lcD.set(m_bodyD.m_sweep.localCenter)
        m_mA = m_bodyA.m_invMass
        m_mB = m_bodyB.m_invMass
        m_mC = m_bodyC.m_invMass
        m_mD = m_bodyD.m_invMass
        m_iA = m_bodyA.m_invI
        m_iB = m_bodyB.m_invI
        m_iC = m_bodyC.m_invI
        m_iD = m_bodyD.m_invI

        val aA = data.positions[m_indexA].a
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val aB = data.positions[m_indexB].a
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w
        val aC = data.positions[m_indexC].a
        val vC = data.velocities[m_indexC].v
        var wC = data.velocities[m_indexC].w
        val aD = data.positions[m_indexD].a
        val vD = data.velocities[m_indexD].v
        var wD = data.velocities[m_indexD].w

        val qA = pool.popRot()
        val qB = pool.popRot()
        val qC = pool.popRot()
        val qD = pool.popRot()
        qA.set(aA)
        qB.set(aB)
        qC.set(aC)
        qD.set(aD)

        m_mass = 0f
        val temp = pool.popVec2()
        if (m_typeA == JointType.REVOLUTE) {
            m_JvAC.setZero()
            m_JwA = 1f
            m_JwC = 1f
            m_mass += m_iA + m_iC
        } else {
            val rC = pool.popVec2()
            val rA = pool.popVec2()
            Rot.mulToOutUnsafe(qC, m_localAxisC, m_JvAC)
            Rot.mulToOutUnsafe(qC, temp.set(m_localAnchorC).subLocal(m_lcC), rC)
            Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_lcA), rA)
            m_JwC = Vec2.cross(rC, m_JvAC)
            m_JwA = Vec2.cross(rA, m_JvAC)
            m_mass += m_mC + m_mA + m_iC * m_JwC * m_JwC + m_iA * m_JwA * m_JwA
            pool.pushVec2(2)
        }

        if (m_typeB == JointType.REVOLUTE) {
            m_JvBD.setZero()
            m_JwB = m_ratio
            m_JwD = m_ratio
            m_mass += m_ratio * m_ratio * (m_iB + m_iD)
        } else {
            val u = pool.popVec2()
            val rD = pool.popVec2()
            val rB = pool.popVec2()
            Rot.mulToOutUnsafe(qD, m_localAxisD, u)
            Rot.mulToOutUnsafe(qD, temp.set(m_localAnchorD).subLocal(m_lcD), rD)
            Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_lcB), rB)
            m_JvBD.set(u).mulLocal(m_ratio)
            m_JwD = m_ratio * Vec2.cross(rD, u)
            m_JwB = m_ratio * Vec2.cross(rB, u)
            m_mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * m_JwD * m_JwD + m_iB * m_JwB * m_JwB
            pool.pushVec2(3)
        }

        m_mass = if (m_mass > 0f) 1f / m_mass else 0f
        if (step.warmStarting) {
            vA.x += (m_mA * m_impulse) * m_JvAC.x
            vA.y += (m_mA * m_impulse) * m_JvAC.y
            wA += m_iA * m_impulse * m_JwA
            vB.x += (m_mB * m_impulse) * m_JvBD.x
            vB.y += (m_mB * m_impulse) * m_JvBD.y
            wB += m_iB * m_impulse * m_JwB
            vC.x -= (m_mC * m_impulse) * m_JvAC.x
            vC.y -= (m_mC * m_impulse) * m_JvAC.y
            wC -= m_iC * m_impulse * m_JwC
            vD.x -= (m_mD * m_impulse) * m_JvBD.x
            vD.y -= (m_mD * m_impulse) * m_JvBD.y
            wD -= m_iD * m_impulse * m_JwD
        } else {
            m_impulse = 0f
        }

        pool.pushVec2(1)
        pool.pushRot(4)
        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
        data.velocities[m_indexC].w = wC
        data.velocities[m_indexD].w = wD
    }

    override fun solveVelocityConstraints(data: SolverData) {
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w
        val vC = data.velocities[m_indexC].v
        var wC = data.velocities[m_indexC].w
        val vD = data.velocities[m_indexD].v
        var wD = data.velocities[m_indexD].w

        val temp1 = pool.popVec2()
        val temp2 = pool.popVec2()
        var cDot = Vec2.dot(m_JvAC, temp1.set(vA).subLocal(vC)) + Vec2.dot(m_JvBD, temp2.set(vB).subLocal(vD))
        cDot += (m_JwA * wA - m_JwC * wC) + (m_JwB * wB - m_JwD * wD)
        pool.pushVec2(2)

        val impulse = -m_mass * cDot
        m_impulse += impulse

        vA.x += (m_mA * impulse) * m_JvAC.x
        vA.y += (m_mA * impulse) * m_JvAC.y
        wA += m_iA * impulse * m_JwA
        vB.x += (m_mB * impulse) * m_JvBD.x
        vB.y += (m_mB * impulse) * m_JvBD.y
        wB += m_iB * impulse * m_JwB
        vC.x -= (m_mC * impulse) * m_JvAC.x
        vC.y -= (m_mC * impulse) * m_JvAC.y
        wC -= m_iC * impulse * m_JwC
        vD.x -= (m_mD * impulse) * m_JvBD.x
        vD.y -= (m_mD * impulse) * m_JvBD.y
        wD -= m_iD * impulse * m_JwD

        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
        data.velocities[m_indexC].w = wC
        data.velocities[m_indexD].w = wD
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        val cA = data.positions[m_indexA].c
        var aA = data.positions[m_indexA].a
        val cB = data.positions[m_indexB].c
        var aB = data.positions[m_indexB].a
        val cC = data.positions[m_indexC].c
        var aC = data.positions[m_indexC].a
        val cD = data.positions[m_indexD].c
        var aD = data.positions[m_indexD].a

        val qA = pool.popRot()
        val qB = pool.popRot()
        val qC = pool.popRot()
        val qD = pool.popRot()
        qA.set(aA)
        qB.set(aB)
        qC.set(aC)
        qD.set(aD)

        var linearError = 0f
        val temp = pool.popVec2()
        val jvAC = pool.popVec2()
        val jvBD = pool.popVec2()
        var jwA: Float
        var jwB: Float
        var jwC: Float
        var jwD: Float
        var mass = 0f
        val coordinateA: Float
        val coordinateB: Float

        if (m_typeA == JointType.REVOLUTE) {
            jvAC.setZero()
            jwA = 1f
            jwC = 1f
            mass += m_iA + m_iC
            coordinateA = aA - aC - m_referenceAngleA
        } else {
            val rC = pool.popVec2()
            val rA = pool.popVec2()
            val pC = pool.popVec2()
            val pA = pool.popVec2()
            Rot.mulToOutUnsafe(qC, m_localAxisC, jvAC)
            Rot.mulToOutUnsafe(qC, temp.set(m_localAnchorC).subLocal(m_lcC), rC)
            Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_lcA), rA)
            jwC = Vec2.cross(rC, jvAC)
            jwA = Vec2.cross(rA, jvAC)
            mass += m_mC + m_mA + m_iC * jwC * jwC + m_iA * jwA * jwA
            pC.set(m_localAnchorC).subLocal(m_lcC)
            Rot.mulTransUnsafe(qC, temp.set(rA).addLocal(cA).subLocal(cC), pA)
            coordinateA = Vec2.dot(pA.subLocal(pC), m_localAxisC)
            pool.pushVec2(4)
        }

        if (m_typeB == JointType.REVOLUTE) {
            jvBD.setZero()
            jwB = m_ratio
            jwD = m_ratio
            mass += m_ratio * m_ratio * (m_iB + m_iD)
            coordinateB = aB - aD - m_referenceAngleB
        } else {
            val u = pool.popVec2()
            val rD = pool.popVec2()
            val rB = pool.popVec2()
            val pD = pool.popVec2()
            val pB = pool.popVec2()
            Rot.mulToOutUnsafe(qD, m_localAxisD, u)
            Rot.mulToOutUnsafe(qD, temp.set(m_localAnchorD).subLocal(m_lcD), rD)
            Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_lcB), rB)
            jvBD.set(u).mulLocal(m_ratio)
            jwD = m_ratio * Vec2.cross(rD, u)
            jwB = m_ratio * Vec2.cross(rB, u)
            mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * jwD * jwD + m_iB * jwB * jwB
            pD.set(m_localAnchorD).subLocal(m_lcD)
            Rot.mulTransUnsafe(qD, temp.set(rB).addLocal(cB).subLocal(cD), pB)
            coordinateB = Vec2.dot(pB.subLocal(pD), m_localAxisD)
            pool.pushVec2(5)
        }

        val c = (coordinateA + m_ratio * coordinateB) - m_constant
        val impulse = if (mass > 0f) -c / mass else 0f
        pool.pushVec2(3)
        pool.pushRot(4)

        cA.x += (m_mA * impulse) * jvAC.x
        cA.y += (m_mA * impulse) * jvAC.y
        aA += m_iA * impulse * jwA
        cB.x += (m_mB * impulse) * jvBD.x
        cB.y += (m_mB * impulse) * jvBD.y
        aB += m_iB * impulse * jwB
        cC.x -= (m_mC * impulse) * jvAC.x
        cC.y -= (m_mC * impulse) * jvAC.y
        aC -= m_iC * impulse * jwC
        cD.x -= (m_mD * impulse) * jvBD.x
        cD.y -= (m_mD * impulse) * jvBD.y
        aD -= m_iD * impulse * jwD

        data.positions[m_indexA].a = aA
        data.positions[m_indexB].a = aB
        data.positions[m_indexC].a = aC
        data.positions[m_indexD].a = aD
        return linearError < Settings.linearSlop
    }
}
