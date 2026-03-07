package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Mat22
import xyz.thewind.kbox2d.common.Mat33
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.common.Vec3
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.pooling.IWorldPool

class PrismaticJoint(
    pool: IWorldPool,
    def: PrismaticJointDef,
) : Joint(pool, def) {
    private val m_localAnchorA: Vec2 = def.localAnchorA.clone()
    private val m_localAnchorB: Vec2 = def.localAnchorB.clone()
    private val m_localXAxisA: Vec2 = def.localAxisA.clone().also { it.normalize() }
    private val m_localYAxisA: Vec2 = Vec2().also { Vec2.crossToOutUnsafe(1f, m_localXAxisA, it) }
    private var m_referenceAngle: Float = def.referenceAngle
    private val m_impulse: Vec3 = Vec3()
    private var m_motorImpulse: Float = 0f
    private var m_lowerTranslation: Float = def.lowerTranslation
    private var m_upperTranslation: Float = def.upperTranslation
    private var m_maxMotorForce: Float = def.maxMotorForce
    private var m_motorSpeed: Float = def.motorSpeed
    private var m_enableLimit: Boolean = def.enableLimit
    private var m_enableMotor: Boolean = def.enableMotor
    private var m_limitState: LimitState = LimitState.INACTIVE
    private var m_indexA: Int = 0
    private var m_indexB: Int = 0
    private val m_localCenterA: Vec2 = Vec2()
    private val m_localCenterB: Vec2 = Vec2()
    private var m_invMassA: Float = 0f
    private var m_invMassB: Float = 0f
    private var m_invIA: Float = 0f
    private var m_invIB: Float = 0f
    private val m_axis: Vec2 = Vec2()
    private val m_perp: Vec2 = Vec2()
    private var m_s1: Float = 0f
    private var m_s2: Float = 0f
    private var m_a1: Float = 0f
    private var m_a2: Float = 0f
    private val m_K: Mat33 = Mat33()
    private var m_motorMass: Float = 0f

    fun getLocalAnchorA(): Vec2 = m_localAnchorA
    fun getLocalAnchorB(): Vec2 = m_localAnchorB
    fun getReferenceAngle(): Float = m_referenceAngle
    fun getLocalAxisA(): Vec2 = m_localXAxisA

    override fun getAnchorA(out: Vec2) {
        m_bodyA.getWorldPointToOut(m_localAnchorA, out)
    }

    override fun getAnchorB(out: Vec2) {
        m_bodyB.getWorldPointToOut(m_localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        val temp = pool.popVec2()
        temp.set(m_axis).mulLocal(m_motorImpulse + m_impulse.z)
        out.set(m_perp).mulLocal(m_impulse.x).addLocal(temp).mulLocal(invDt)
        pool.pushVec2(1)
    }

    override fun getReactionTorque(invDt: Float): Float = invDt * m_impulse.y

    fun getJointSpeed(): Float {
        val bA = m_bodyA
        val bB = m_bodyB
        val temp = pool.popVec2()
        val rA = pool.popVec2()
        val rB = pool.popVec2()
        val p1 = pool.popVec2()
        val p2 = pool.popVec2()
        val d = pool.popVec2()
        val axis = pool.popVec2()
        val temp2 = pool.popVec2()
        val temp3 = pool.popVec2()

        temp.set(m_localAnchorA).subLocal(bA.m_sweep.localCenter)
        Rot.mulToOutUnsafe(bA.m_xf.q, temp, rA)
        temp.set(m_localAnchorB).subLocal(bB.m_sweep.localCenter)
        Rot.mulToOutUnsafe(bB.m_xf.q, temp, rB)
        p1.set(bA.m_sweep.c).addLocal(rA)
        p2.set(bB.m_sweep.c).addLocal(rB)
        d.set(p2).subLocal(p1)
        bA.getWorldVectorToOut(m_localXAxisA, axis)

        val vA = bA.m_linearVelocity
        val vB = bB.m_linearVelocity
        val wA = bA.m_angularVelocity
        val wB = bB.m_angularVelocity
        Vec2.crossToOutUnsafe(wA, axis, temp)
        Vec2.crossToOutUnsafe(wB, rB, temp2)
        Vec2.crossToOutUnsafe(wA, rA, temp3)
        temp2.addLocal(vB).subLocal(vA).subLocal(temp3)
        val speed = Vec2.dot(d, temp) + Vec2.dot(axis, temp2)

        pool.pushVec2(9)
        return speed
    }

    fun getJointTranslation(): Float {
        val pA = pool.popVec2()
        val pB = pool.popVec2()
        val axis = pool.popVec2()
        m_bodyA.getWorldPointToOut(m_localAnchorA, pA)
        m_bodyB.getWorldPointToOut(m_localAnchorB, pB)
        m_bodyA.getWorldVectorToOut(m_localXAxisA, axis)
        pB.subLocal(pA)
        val translation = Vec2.dot(pB, axis)
        pool.pushVec2(3)
        return translation
    }

    fun isLimitEnabled(): Boolean = m_enableLimit

    fun enableLimit(flag: Boolean) {
        if (flag != m_enableLimit) {
            m_bodyA.setAwake(true)
            m_bodyB.setAwake(true)
            m_enableLimit = flag
            m_impulse.z = 0f
        }
    }

    fun getLowerLimit(): Float = m_lowerTranslation

    fun getUpperLimit(): Float = m_upperTranslation

    fun setLimits(lower: Float, upper: Float) {
        require(lower <= upper)
        if (lower != m_lowerTranslation || upper != m_upperTranslation) {
            m_bodyA.setAwake(true)
            m_bodyB.setAwake(true)
            m_lowerTranslation = lower
            m_upperTranslation = upper
            m_impulse.z = 0f
        }
    }

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

    fun setMaxMotorForce(force: Float) {
        m_bodyA.setAwake(true)
        m_bodyB.setAwake(true)
        m_maxMotorForce = force
    }

    fun getMotorForce(invDt: Float): Float = m_motorImpulse * invDt

    fun getMaxMotorForce(): Float = m_maxMotorForce

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
        val d = pool.popVec2()
        val temp = pool.popVec2()
        val rA = pool.popVec2()
        val rB = pool.popVec2()
        qA.set(aA)
        qB.set(aB)

        Rot.mulToOutUnsafe(qA, d.set(m_localAnchorA).subLocal(m_localCenterA), rA)
        Rot.mulToOutUnsafe(qB, d.set(m_localAnchorB).subLocal(m_localCenterB), rB)
        d.set(cB).subLocal(cA).addLocal(rB).subLocal(rA)

        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB

        Rot.mulToOutUnsafe(qA, m_localXAxisA, m_axis)
        temp.set(d).addLocal(rA)
        m_a1 = Vec2.cross(temp, m_axis)
        m_a2 = Vec2.cross(rB, m_axis)
        m_motorMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2
        if (m_motorMass > 0f) {
            m_motorMass = 1f / m_motorMass
        }

        Rot.mulToOutUnsafe(qA, m_localYAxisA, m_perp)
        temp.set(d).addLocal(rA)
        m_s1 = Vec2.cross(temp, m_perp)
        m_s2 = Vec2.cross(rB, m_perp)

        val k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2
        val k12 = iA * m_s1 + iB * m_s2
        val k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2
        var k22 = iA + iB
        if (k22 == 0f) {
            k22 = 1f
        }
        val k23 = iA * m_a1 + iB * m_a2
        val k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2
        m_K.ex.set(k11, k12, k13)
        m_K.ey.set(k12, k22, k23)
        m_K.ez.set(k13, k23, k33)

        if (m_enableLimit) {
            val jointTranslation = Vec2.dot(m_axis, d)
            when {
                MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2f * Settings.linearSlop -> {
                    m_limitState = LimitState.EQUAL
                }
                jointTranslation <= m_lowerTranslation -> {
                    if (m_limitState != LimitState.AT_LOWER) {
                        m_impulse.z = 0f
                    }
                    m_limitState = LimitState.AT_LOWER
                }
                jointTranslation >= m_upperTranslation -> {
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
            m_impulse.z = 0f
        }

        if (!m_enableMotor) {
            m_motorImpulse = 0f
        }

        if (step.warmStarting) {
            m_impulse.mulLocal(step.dtRatio)
            m_motorImpulse *= step.dtRatio

            val p = pool.popVec2()
            temp.set(m_axis).mulLocal(m_motorImpulse + m_impulse.z)
            p.set(m_perp).mulLocal(m_impulse.x).addLocal(temp)
            val la = m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1
            val lb = m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2

            vA.x -= mA * p.x
            vA.y -= mA * p.y
            wA -= iA * la
            vB.x += mB * p.x
            vB.y += mB * p.y
            wB += iB * lb
            pool.pushVec2(1)
        } else {
            m_impulse.setZero()
            m_motorImpulse = 0f
        }

        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
        pool.pushRot(2)
        pool.pushVec2(4)
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
        val temp = pool.popVec2()

        if (m_enableMotor && m_limitState != LimitState.EQUAL) {
            temp.set(vB).subLocal(vA)
            var impulse = m_motorMass * (m_motorSpeed - (Vec2.dot(m_axis, temp) + m_a2 * wB - m_a1 * wA))
            val oldImpulse = m_motorImpulse
            val maxImpulse = step.dt * m_maxMotorForce
            m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse)
            impulse = m_motorImpulse - oldImpulse

            val p = pool.popVec2()
            p.set(m_axis).mulLocal(impulse)
            val la = impulse * m_a1
            val lb = impulse * m_a2
            vA.x -= mA * p.x
            vA.y -= mA * p.y
            wA -= iA * la
            vB.x += mB * p.x
            vB.y += mB * p.y
            wB += iB * lb
            pool.pushVec2(1)
        }

        val cDot1 = pool.popVec2()
        temp.set(vB).subLocal(vA)
        cDot1.x = Vec2.dot(m_perp, temp) + m_s2 * wB - m_s1 * wA
        cDot1.y = wB - wA

        if (m_enableLimit && m_limitState != LimitState.INACTIVE) {
            temp.set(vB).subLocal(vA)
            val cDot2 = Vec2.dot(m_axis, temp) + m_a2 * wB - m_a1 * wA
            val cDot = pool.popVec3()
            cDot.set(cDot1.x, cDot1.y, cDot2)

            val f1 = pool.popVec3()
            val df = pool.popVec3()
            f1.set(m_impulse)
            m_K.solve33ToOut(cDot.negateLocal(), df)
            m_impulse.addLocal(df)

            if (m_limitState == LimitState.AT_LOWER) {
                m_impulse.z = MathUtils.max(m_impulse.z, 0f)
            } else if (m_limitState == LimitState.AT_UPPER) {
                m_impulse.z = MathUtils.min(m_impulse.z, 0f)
            }

            val b = pool.popVec2()
            val f2r = pool.popVec2()
            temp.set(m_K.ez.x, m_K.ez.y).mulLocal(m_impulse.z - f1.z)
            b.set(cDot1).negateLocal().subLocal(temp)
            m_K.solve22ToOut(b, f2r)
            f2r.addLocal(f1.x, f1.y)
            m_impulse.x = f2r.x
            m_impulse.y = f2r.y
            df.set(m_impulse).subLocal(f1)

            val p = pool.popVec2()
            temp.set(m_axis).mulLocal(df.z)
            p.set(m_perp).mulLocal(df.x).addLocal(temp)
            val la = df.x * m_s1 + df.y + df.z * m_a1
            val lb = df.x * m_s2 + df.y + df.z * m_a2
            vA.x -= mA * p.x
            vA.y -= mA * p.y
            wA -= iA * la
            vB.x += mB * p.x
            vB.y += mB * p.y
            wB += iB * lb

            pool.pushVec2(3)
            pool.pushVec3(3)
        } else {
            val df = pool.popVec2()
            m_K.solve22ToOut(cDot1.negateLocal(), df)
            m_impulse.x += df.x
            m_impulse.y += df.y

            val p = pool.popVec2()
            p.set(m_perp).mulLocal(df.x)
            val la = df.x * m_s1 + df.y
            val lb = df.x * m_s2 + df.y
            vA.x -= mA * p.x
            vA.y -= mA * p.y
            wA -= iA * la
            vB.x += mB * p.x
            vB.y += mB * p.y
            wB += iB * lb
            pool.pushVec2(2)
        }

        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
        pool.pushVec2(2)
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        val qA = pool.popRot()
        val qB = pool.popRot()
        val rA = pool.popVec2()
        val rB = pool.popVec2()
        val d = pool.popVec2()
        val axis = pool.popVec2()
        val perp = pool.popVec2()
        val temp = pool.popVec2()
        val c1 = pool.popVec2()
        val impulse = pool.popVec3()

        val cA = data.positions[m_indexA].c
        var aA = data.positions[m_indexA].a
        val cB = data.positions[m_indexB].c
        var aB = data.positions[m_indexB].a

        qA.set(aA)
        qB.set(aB)

        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB

        Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), rA)
        Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), rB)
        d.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)

        Rot.mulToOutUnsafe(qA, m_localXAxisA, axis)
        val a1 = Vec2.cross(temp.set(d).addLocal(rA), axis)
        val a2 = Vec2.cross(rB, axis)
        Rot.mulToOutUnsafe(qA, m_localYAxisA, perp)
        val s1 = Vec2.cross(temp.set(d).addLocal(rA), perp)
        val s2 = Vec2.cross(rB, perp)

        c1.x = Vec2.dot(perp, d)
        c1.y = aB - aA - m_referenceAngle
        var linearError = MathUtils.abs(c1.x)
        val angularError = MathUtils.abs(c1.y)

        var active = false
        var c2 = 0f
        if (m_enableLimit) {
            val translation = Vec2.dot(axis, d)
            when {
                MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2f * Settings.linearSlop -> {
                    c2 = MathUtils.clamp(translation, -Settings.maxLinearCorrection, Settings.maxLinearCorrection)
                    linearError = MathUtils.max(linearError, MathUtils.abs(translation))
                    active = true
                }
                translation <= m_lowerTranslation -> {
                    c2 = MathUtils.clamp(
                        translation - m_lowerTranslation + Settings.linearSlop,
                        -Settings.maxLinearCorrection,
                        0f,
                    )
                    linearError = MathUtils.max(linearError, m_lowerTranslation - translation)
                    active = true
                }
                translation >= m_upperTranslation -> {
                    c2 = MathUtils.clamp(
                        translation - m_upperTranslation - Settings.linearSlop,
                        0f,
                        Settings.maxLinearCorrection,
                    )
                    linearError = MathUtils.max(linearError, translation - m_upperTranslation)
                    active = true
                }
            }
        }

        if (active) {
            val k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2
            val k12 = iA * s1 + iB * s2
            val k13 = iA * s1 * a1 + iB * s2 * a2
            var k22 = iA + iB
            if (k22 == 0f) {
                k22 = 1f
            }
            val k23 = iA * a1 + iB * a2
            val k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2
            val k = pool.popMat33()
            k.ex.set(k11, k12, k13)
            k.ey.set(k12, k22, k23)
            k.ez.set(k13, k23, k33)

            val c = pool.popVec3()
            c.set(c1.x, c1.y, c2)
            k.solve33ToOut(c.negateLocal(), impulse)
            pool.pushVec3(1)
            pool.pushMat33(1)
        } else {
            val k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2
            val k12 = iA * s1 + iB * s2
            var k22 = iA + iB
            if (k22 == 0f) {
                k22 = 1f
            }
            val k = pool.popMat22()
            k.ex.set(k11, k12)
            k.ey.set(k12, k22)
            k.solveToOut(c1.negateLocal(), temp)
            impulse.x = temp.x
            impulse.y = temp.y
            impulse.z = 0f
            pool.pushMat22(1)
        }

        val px = impulse.x * perp.x + impulse.z * axis.x
        val py = impulse.x * perp.y + impulse.z * axis.y
        val la = impulse.x * s1 + impulse.y + impulse.z * a1
        val lb = impulse.x * s2 + impulse.y + impulse.z * a2

        cA.x -= mA * px
        cA.y -= mA * py
        aA -= iA * la
        cB.x += mB * px
        cB.y += mB * py
        aB += iB * lb

        data.positions[m_indexA].a = aA
        data.positions[m_indexB].a = aB

        pool.pushVec2(7)
        pool.pushVec3(1)
        pool.pushRot(2)
        return linearError <= Settings.linearSlop && angularError <= Settings.angularSlop
    }
}
