package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.pooling.IWorldPool

class WheelJoint(
    pool: IWorldPool,
    def: WheelJointDef,
) : Joint(pool, def) {
    private var m_frequencyHz: Float = def.frequencyHz
    private var m_dampingRatio: Float = def.dampingRatio
    private val m_localAnchorA: Vec2 = def.localAnchorA.clone()
    private val m_localAnchorB: Vec2 = def.localAnchorB.clone()
    private val m_localXAxisA: Vec2 = def.localAxisA.clone()
    private val m_localYAxisA: Vec2 = Vec2().also { Vec2.crossToOutUnsafe(1f, m_localXAxisA, it) }
    private var m_impulse: Float = 0f
    private var m_motorImpulse: Float = 0f
    private var m_springImpulse: Float = 0f
    private var m_maxMotorTorque: Float = def.maxMotorTorque
    private var m_motorSpeed: Float = def.motorSpeed
    private var m_enableMotor: Boolean = def.enableMotor
    private var m_indexA: Int = 0
    private var m_indexB: Int = 0
    private val m_localCenterA: Vec2 = Vec2()
    private val m_localCenterB: Vec2 = Vec2()
    private var m_invMassA: Float = 0f
    private var m_invMassB: Float = 0f
    private var m_invIA: Float = 0f
    private var m_invIB: Float = 0f
    private val m_ax: Vec2 = Vec2()
    private val m_ay: Vec2 = Vec2()
    private var m_sAx: Float = 0f
    private var m_sBx: Float = 0f
    private var m_sAy: Float = 0f
    private var m_sBy: Float = 0f
    private var m_mass: Float = 0f
    private var m_motorMass: Float = 0f
    private var m_springMass: Float = 0f
    private var m_bias: Float = 0f
    private var m_gamma: Float = 0f
    private val rA: Vec2 = Vec2()
    private val rB: Vec2 = Vec2()
    private val d: Vec2 = Vec2()

    fun getLocalAnchorA(): Vec2 = m_localAnchorA
    fun getLocalAnchorB(): Vec2 = m_localAnchorB
    fun getLocalAxisA(): Vec2 = m_localXAxisA

    override fun getAnchorA(out: Vec2) {
        m_bodyA.getWorldPointToOut(m_localAnchorA, out)
    }

    override fun getAnchorB(out: Vec2) {
        m_bodyB.getWorldPointToOut(m_localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        val temp = pool.popVec2()
        temp.set(m_ay).mulLocal(m_impulse)
        out.set(m_ax).mulLocal(m_springImpulse).addLocal(temp).mulLocal(invDt)
        pool.pushVec2(1)
    }

    override fun getReactionTorque(invDt: Float): Float = invDt * m_motorImpulse

    fun getJointTranslation(): Float {
        val p1 = pool.popVec2()
        val p2 = pool.popVec2()
        val axis = pool.popVec2()
        m_bodyA.getWorldPointToOut(m_localAnchorA, p1)
        m_bodyB.getWorldPointToOut(m_localAnchorB, p2)
        p2.subLocal(p1)
        m_bodyA.getWorldVectorToOut(m_localXAxisA, axis)
        val translation = Vec2.dot(p2, axis)
        pool.pushVec2(3)
        return translation
    }

    fun getJointSpeed(): Float = m_bodyA.m_angularVelocity - m_bodyB.m_angularVelocity

    fun isMotorEnabled(): Boolean = m_enableMotor

    fun enableMotor(flag: Boolean) {
        m_bodyA.setAwake(true)
        m_bodyB.setAwake(true)
        m_enableMotor = flag
    }

    fun setMotorSpeed(speed: Float) {
        m_bodyA.setAwake(true)
        m_bodyB.setAwake(true)
        m_motorSpeed = speed
    }

    fun getMotorSpeed(): Float = m_motorSpeed

    fun getMaxMotorTorque(): Float = m_maxMotorTorque

    fun setMaxMotorTorque(torque: Float) {
        m_bodyA.setAwake(true)
        m_bodyB.setAwake(true)
        m_maxMotorTorque = torque
    }

    fun getMotorTorque(invDt: Float): Float = m_motorImpulse * invDt

    fun setSpringFrequencyHz(hz: Float) {
        m_frequencyHz = hz
    }

    fun getSpringFrequencyHz(): Float = m_frequencyHz

    fun setSpringDampingRatio(ratio: Float) {
        m_dampingRatio = ratio
    }

    fun getSpringDampingRatio(): Float = m_dampingRatio

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

        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB
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

        Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), rA)
        Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), rB)
        d.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)

        Rot.mulToOut(qA, m_localYAxisA, m_ay)
        m_sAy = Vec2.cross(temp.set(d).addLocal(rA), m_ay)
        m_sBy = Vec2.cross(rB, m_ay)
        m_mass = mA + mB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy
        if (m_mass > 0f) {
            m_mass = 1f / m_mass
        }

        m_springMass = 0f
        m_bias = 0f
        m_gamma = 0f
        if (m_frequencyHz > 0f) {
            Rot.mulToOut(qA, m_localXAxisA, m_ax)
            m_sAx = Vec2.cross(temp.set(d).addLocal(rA), m_ax)
            m_sBx = Vec2.cross(rB, m_ax)
            val invMass = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx
            if (invMass > 0f) {
                m_springMass = 1f / invMass
                val c = Vec2.dot(d, m_ax)
                val omega = 2f * MathUtils.PI * m_frequencyHz
                val damping = 2f * m_springMass * m_dampingRatio * omega
                val stiffness = m_springMass * omega * omega
                val h = step.dt
                m_gamma = h * (damping + h * stiffness)
                if (m_gamma > 0f) {
                    m_gamma = 1f / m_gamma
                }
                m_bias = c * h * stiffness * m_gamma
                m_springMass = invMass + m_gamma
                if (m_springMass > 0f) {
                    m_springMass = 1f / m_springMass
                }
            }
        } else {
            m_springImpulse = 0f
        }

        if (m_enableMotor) {
            m_motorMass = iA + iB
            if (m_motorMass > 0f) {
                m_motorMass = 1f / m_motorMass
            }
        } else {
            m_motorMass = 0f
            m_motorImpulse = 0f
        }

        if (step.warmStarting) {
            val p = pool.popVec2()
            m_impulse *= step.dtRatio
            m_springImpulse *= step.dtRatio
            m_motorImpulse *= step.dtRatio
            p.x = m_impulse * m_ay.x + m_springImpulse * m_ax.x
            p.y = m_impulse * m_ay.y + m_springImpulse * m_ax.y
            val la = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse
            val lb = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse
            vA.x -= m_invMassA * p.x
            vA.y -= m_invMassA * p.y
            wA -= m_invIA * la
            vB.x += m_invMassB * p.x
            vB.y += m_invMassB * p.y
            wB += m_invIB * lb
            pool.pushVec2(1)
        } else {
            m_impulse = 0f
            m_springImpulse = 0f
            m_motorImpulse = 0f
        }

        pool.pushRot(2)
        pool.pushVec2(1)
        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
    }

    override fun solveVelocityConstraints(data: SolverData) {
        val step = data.step!!
        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w
        val temp = pool.popVec2()
        val p = pool.popVec2()

        run {
            val cDot = Vec2.dot(m_ax, temp.set(vB).subLocal(vA)) + m_sBx * wB - m_sAx * wA
            val impulse = -m_springMass * (cDot + m_bias + m_gamma * m_springImpulse)
            m_springImpulse += impulse
            p.x = impulse * m_ax.x
            p.y = impulse * m_ax.y
            val la = impulse * m_sAx
            val lb = impulse * m_sBx
            vA.x -= mA * p.x
            vA.y -= mA * p.y
            wA -= iA * la
            vB.x += mB * p.x
            vB.y += mB * p.y
            wB += iB * lb
        }

        run {
            val cDot = wB - wA - m_motorSpeed
            var impulse = -m_motorMass * cDot
            val oldImpulse = m_motorImpulse
            val maxImpulse = step.dt * m_maxMotorTorque
            m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse)
            impulse = m_motorImpulse - oldImpulse
            wA -= iA * impulse
            wB += iB * impulse
        }

        run {
            val cDot = Vec2.dot(m_ay, temp.set(vB).subLocal(vA)) + m_sBy * wB - m_sAy * wA
            val impulse = -m_mass * cDot
            m_impulse += impulse
            p.x = impulse * m_ay.x
            p.y = impulse * m_ay.y
            val la = impulse * m_sAy
            val lb = impulse * m_sBy
            vA.x -= mA * p.x
            vA.y -= mA * p.y
            wA -= iA * la
            vB.x += mB * p.x
            vB.y += mB * p.y
            wB += iB * lb
        }

        pool.pushVec2(2)
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
        val temp = pool.popVec2()
        qA.set(aA)
        qB.set(aB)

        Rot.mulToOut(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), rA)
        Rot.mulToOut(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), rB)
        d.set(cB).subLocal(cA).addLocal(rB).subLocal(rA)

        val ay = pool.popVec2()
        Rot.mulToOut(qA, m_localYAxisA, ay)
        val sAy = Vec2.cross(temp.set(d).addLocal(rA), ay)
        val sBy = Vec2.cross(rB, ay)
        val c = Vec2.dot(d, ay)
        val k = m_invMassA + m_invMassB + m_invIA * sAy * sAy + m_invIB * sBy * sBy
        val impulse = if (k != 0f) -c / k else 0f

        val p = pool.popVec2()
        p.x = impulse * ay.x
        p.y = impulse * ay.y
        val la = impulse * sAy
        val lb = impulse * sBy
        cA.x -= m_invMassA * p.x
        cA.y -= m_invMassA * p.y
        aA -= m_invIA * la
        cB.x += m_invMassB * p.x
        cB.y += m_invMassB * p.y
        aB += m_invIB * lb

        pool.pushVec2(3)
        pool.pushRot(2)
        data.positions[m_indexA].a = aA
        data.positions[m_indexB].a = aB
        return MathUtils.abs(c) <= Settings.linearSlop
    }
}
