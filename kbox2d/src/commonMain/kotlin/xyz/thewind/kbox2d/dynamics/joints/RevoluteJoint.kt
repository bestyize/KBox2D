package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Mat22
import xyz.thewind.kbox2d.common.Mat33
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.common.Vec3
import xyz.thewind.kbox2d.dynamics.Body
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.pooling.IWorldPool

class RevoluteJoint(
    pool: IWorldPool,
    def: RevoluteJointDef,
) : Joint(pool, def) {
    private val m_localAnchorA: Vec2 = def.localAnchorA.clone()
    private val m_localAnchorB: Vec2 = def.localAnchorB.clone()
    private val m_impulse: Vec3 = Vec3()
    private var m_motorImpulse: Float = 0f
    private var m_enableMotor: Boolean = def.enableMotor
    private var m_maxMotorTorque: Float = def.maxMotorTorque
    private var m_motorSpeed: Float = def.motorSpeed
    private var m_enableLimit: Boolean = def.enableLimit
    private var m_referenceAngle: Float = def.referenceAngle
    private var m_lowerAngle: Float = def.lowerAngle
    private var m_upperAngle: Float = def.upperAngle
    private var m_indexA: Int = 0
    private var m_indexB: Int = 0
    private val m_rA: Vec2 = Vec2()
    private val m_rB: Vec2 = Vec2()
    private val m_localCenterA: Vec2 = Vec2()
    private val m_localCenterB: Vec2 = Vec2()
    private var m_invMassA: Float = 0f
    private var m_invMassB: Float = 0f
    private var m_invIA: Float = 0f
    private var m_invIB: Float = 0f
    private val m_mass: Mat33 = Mat33()
    private var m_motorMass: Float = 0f
    private var m_limitState: LimitState = LimitState.INACTIVE

    fun getLocalAnchorA(): Vec2 = m_localAnchorA
    fun getLocalAnchorB(): Vec2 = m_localAnchorB
    fun getReferenceAngle(): Float = m_referenceAngle

    override fun getAnchorA(out: Vec2) {
        m_bodyA.getWorldPointToOut(m_localAnchorA, out)
    }

    override fun getAnchorB(out: Vec2) {
        m_bodyB.getWorldPointToOut(m_localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.set(m_impulse.x, m_impulse.y).mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float = invDt * m_impulse.z

    fun getJointAngle(): Float = m_bodyB.m_sweep.a - m_bodyA.m_sweep.a - m_referenceAngle

    fun getJointSpeed(): Float = m_bodyB.m_angularVelocity - m_bodyA.m_angularVelocity

    fun isMotorEnabled(): Boolean = m_enableMotor

    fun enableMotor(flag: Boolean) {
        m_bodyA.setAwake(true)
        m_bodyB.setAwake(true)
        m_enableMotor = flag
    }

    fun getMotorTorque(invDt: Float): Float = m_motorImpulse * invDt

    fun setMotorSpeed(speed: Float) {
        m_bodyA.setAwake(true)
        m_bodyB.setAwake(true)
        m_motorSpeed = speed
    }

    fun getMotorSpeed(): Float = m_motorSpeed

    fun setMaxMotorTorque(torque: Float) {
        m_bodyA.setAwake(true)
        m_bodyB.setAwake(true)
        m_maxMotorTorque = torque
    }

    fun getMaxMotorTorque(): Float = m_maxMotorTorque

    fun isLimitEnabled(): Boolean = m_enableLimit

    fun enableLimit(flag: Boolean) {
        if (flag != m_enableLimit) {
            m_bodyA.setAwake(true)
            m_bodyB.setAwake(true)
            m_enableLimit = flag
            m_impulse.z = 0f
        }
    }

    fun getLowerLimit(): Float = m_lowerAngle

    fun getUpperLimit(): Float = m_upperAngle

    fun setLimits(lower: Float, upper: Float) {
        require(lower <= upper)
        if (lower != m_lowerAngle || upper != m_upperAngle) {
            m_bodyA.setAwake(true)
            m_bodyB.setAwake(true)
            m_impulse.z = 0f
            m_lowerAngle = lower
            m_upperAngle = upper
        }
    }

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

        val aA = data.positions[m_indexA].a
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
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

        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB
        val fixedRotation = iA + iB == 0f

        m_mass.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB
        m_mass.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB
        m_mass.ez.x = -m_rA.y * iA - m_rB.y * iB
        m_mass.ex.y = m_mass.ey.x
        m_mass.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB
        m_mass.ez.y = m_rA.x * iA + m_rB.x * iB
        m_mass.ex.z = m_mass.ez.x
        m_mass.ey.z = m_mass.ez.y
        m_mass.ez.z = iA + iB

        m_motorMass = iA + iB
        if (m_motorMass > 0f) {
            m_motorMass = 1f / m_motorMass
        }

        if (!m_enableMotor || fixedRotation) {
            m_motorImpulse = 0f
        }

        if (m_enableLimit && !fixedRotation) {
            val jointAngle = aB - aA - m_referenceAngle
            when {
                MathUtils.abs(m_upperAngle - m_lowerAngle) < 2f * Settings.angularSlop -> {
                    m_limitState = LimitState.EQUAL
                }
                jointAngle <= m_lowerAngle -> {
                    if (m_limitState != LimitState.AT_LOWER) {
                        m_impulse.z = 0f
                    }
                    m_limitState = LimitState.AT_LOWER
                }
                jointAngle >= m_upperAngle -> {
                    if (m_limitState != LimitState.AT_UPPER) {
                        m_impulse.z = 0f
                    }
                    m_limitState = LimitState.AT_UPPER
                }
                else -> {
                    m_limitState = LimitState.INACTIVE
                    m_impulse.z = 0f
                }
            }
        } else {
            m_limitState = LimitState.INACTIVE
        }

        if (step.warmStarting) {
            val p = pool.popVec2()
            m_impulse.x *= step.dtRatio
            m_impulse.y *= step.dtRatio
            m_motorImpulse *= step.dtRatio
            p.set(m_impulse.x, m_impulse.y)

            vA.x -= mA * p.x
            vA.y -= mA * p.y
            wA -= iA * (Vec2.cross(m_rA, p) + m_motorImpulse + m_impulse.z)
            vB.x += mB * p.x
            vB.y += mB * p.y
            wB += iB * (Vec2.cross(m_rB, p) + m_motorImpulse + m_impulse.z)
            pool.pushVec2(1)
        } else {
            m_impulse.setZero()
            m_motorImpulse = 0f
        }

        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
        pool.pushVec2(1)
        pool.pushRot(2)
    }

    override fun solveVelocityConstraints(data: SolverData) {
        val step = data.step!!
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w

        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB
        val fixedRotation = iA + iB == 0f

        if (m_enableMotor && m_limitState != LimitState.EQUAL && !fixedRotation) {
            val cDot = wB - wA - m_motorSpeed
            var impulse = -m_motorMass * cDot
            val oldImpulse = m_motorImpulse
            val maxImpulse = step.dt * m_maxMotorTorque
            m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse)
            impulse = m_motorImpulse - oldImpulse
            wA -= iA * impulse
            wB += iB * impulse
        }

        val temp = pool.popVec2()
        if (m_enableLimit && m_limitState != LimitState.INACTIVE && !fixedRotation) {
            val cDot1 = pool.popVec2()
            val cDot = pool.popVec3()

            Vec2.crossToOutUnsafe(wA, m_rA, temp)
            Vec2.crossToOutUnsafe(wB, m_rB, cDot1)
            cDot1.addLocal(vB).subLocal(vA).subLocal(temp)
            cDot.set(cDot1.x, cDot1.y, wB - wA)

            val impulse = pool.popVec3()
            m_mass.solve33ToOut(cDot, impulse)
            impulse.negateLocal()

            when (m_limitState) {
                LimitState.EQUAL -> m_impulse.addLocal(impulse)
                LimitState.AT_LOWER -> {
                    val newImpulse = m_impulse.z + impulse.z
                    if (newImpulse < 0f) {
                        val rhs = pool.popVec2()
                        rhs.set(m_mass.ez.x, m_mass.ez.y).mulLocal(m_impulse.z).subLocal(cDot1)
                        m_mass.solve22ToOut(rhs, temp)
                        impulse.x = temp.x
                        impulse.y = temp.y
                        impulse.z = -m_impulse.z
                        m_impulse.x += temp.x
                        m_impulse.y += temp.y
                        m_impulse.z = 0f
                        pool.pushVec2(1)
                    } else {
                        m_impulse.addLocal(impulse)
                    }
                }
                LimitState.AT_UPPER -> {
                    val newImpulse = m_impulse.z + impulse.z
                    if (newImpulse > 0f) {
                        val rhs = pool.popVec2()
                        rhs.set(m_mass.ez.x, m_mass.ez.y).mulLocal(m_impulse.z).subLocal(cDot1)
                        m_mass.solve22ToOut(rhs, temp)
                        impulse.x = temp.x
                        impulse.y = temp.y
                        impulse.z = -m_impulse.z
                        m_impulse.x += temp.x
                        m_impulse.y += temp.y
                        m_impulse.z = 0f
                        pool.pushVec2(1)
                    } else {
                        m_impulse.addLocal(impulse)
                    }
                }
                LimitState.INACTIVE -> Unit
            }

            val p = pool.popVec2()
            p.set(impulse.x, impulse.y)
            vA.x -= mA * p.x
            vA.y -= mA * p.y
            wA -= iA * (Vec2.cross(m_rA, p) + impulse.z)
            vB.x += mB * p.x
            vB.y += mB * p.y
            wB += iB * (Vec2.cross(m_rB, p) + impulse.z)

            pool.pushVec2(2)
            pool.pushVec3(2)
        } else {
            val cDot = pool.popVec2()
            val impulse = pool.popVec2()
            Vec2.crossToOutUnsafe(wA, m_rA, temp)
            Vec2.crossToOutUnsafe(wB, m_rB, cDot)
            cDot.addLocal(vB).subLocal(vA).subLocal(temp)
            m_mass.solve22ToOut(cDot.negateLocal(), impulse)
            m_impulse.x += impulse.x
            m_impulse.y += impulse.y

            vA.x -= mA * impulse.x
            vA.y -= mA * impulse.y
            wA -= iA * Vec2.cross(m_rA, impulse)
            vB.x += mB * impulse.x
            vB.y += mB * impulse.y
            wB += iB * Vec2.cross(m_rB, impulse)
            pool.pushVec2(2)
        }

        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
        pool.pushVec2(1)
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        val qA = pool.popRot()
        val qB = pool.popRot()
        val cA = data.positions[m_indexA].c
        var aA = data.positions[m_indexA].a
        val cB = data.positions[m_indexB].c
        var aB = data.positions[m_indexB].a

        qA.set(aA)
        qB.set(aB)

        var angularError = 0f
        var positionError = 0f
        val fixedRotation = m_invIA + m_invIB == 0f

        if (m_enableLimit && m_limitState != LimitState.INACTIVE && !fixedRotation) {
            val angle = aB - aA - m_referenceAngle
            val limitImpulse = when (m_limitState) {
                LimitState.EQUAL -> {
                    val c = MathUtils.clamp(
                        angle - m_lowerAngle,
                        -Settings.maxAngularCorrection,
                        Settings.maxAngularCorrection,
                    )
                    angularError = MathUtils.abs(c)
                    -m_motorMass * c
                }
                LimitState.AT_LOWER -> {
                    var c = angle - m_lowerAngle
                    angularError = -c
                    c = MathUtils.clamp(c + Settings.angularSlop, -Settings.maxAngularCorrection, 0f)
                    -m_motorMass * c
                }
                LimitState.AT_UPPER -> {
                    var c = angle - m_upperAngle
                    angularError = c
                    c = MathUtils.clamp(c - Settings.angularSlop, 0f, Settings.maxAngularCorrection)
                    -m_motorMass * c
                }
                LimitState.INACTIVE -> 0f
            }
            aA -= m_invIA * limitImpulse
            aB += m_invIB * limitImpulse
        }

        qA.set(aA)
        qB.set(aB)

        val rA = pool.popVec2()
        val rB = pool.popVec2()
        val c = pool.popVec2()
        val impulse = pool.popVec2()
        Rot.mulToOutUnsafe(qA, c.set(m_localAnchorA).subLocal(m_localCenterA), rA)
        Rot.mulToOutUnsafe(qB, c.set(m_localAnchorB).subLocal(m_localCenterB), rB)
        c.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)
        positionError = c.length()

        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB
        val k = pool.popMat22()
        k.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y
        k.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y
        k.ey.x = k.ex.y
        k.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x
        k.solveToOut(c, impulse)
        impulse.negateLocal()

        cA.x -= mA * impulse.x
        cA.y -= mA * impulse.y
        aA -= iA * Vec2.cross(rA, impulse)
        cB.x += mB * impulse.x
        cB.y += mB * impulse.y
        aB += iB * Vec2.cross(rB, impulse)

        data.positions[m_indexA].a = aA
        data.positions[m_indexB].a = aB

        pool.pushVec2(4)
        pool.pushMat22(1)
        pool.pushRot(2)
        return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop
    }
}
