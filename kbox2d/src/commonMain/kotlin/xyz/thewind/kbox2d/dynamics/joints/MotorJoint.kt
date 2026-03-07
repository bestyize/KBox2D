package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Mat22
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.pooling.IWorldPool

class MotorJoint(
    pool: IWorldPool,
    def: MotorJointDef,
) : Joint(pool, def) {
    private val m_linearOffset: Vec2 = def.linearOffset.clone()
    private var m_angularOffset: Float = def.angularOffset
    private val m_linearImpulse: Vec2 = Vec2()
    private var m_angularImpulse: Float = 0f
    private var m_maxForce: Float = def.maxForce
    private var m_maxTorque: Float = def.maxTorque
    private var m_correctionFactor: Float = def.correctionFactor
    private var m_indexA: Int = 0
    private var m_indexB: Int = 0
    private val m_rA: Vec2 = Vec2()
    private val m_rB: Vec2 = Vec2()
    private val m_localCenterA: Vec2 = Vec2()
    private val m_localCenterB: Vec2 = Vec2()
    private val m_linearError: Vec2 = Vec2()
    private var m_angularError: Float = 0f
    private var m_invMassA: Float = 0f
    private var m_invMassB: Float = 0f
    private var m_invIA: Float = 0f
    private var m_invIB: Float = 0f
    private val m_linearMass: Mat22 = Mat22()
    private var m_angularMass: Float = 0f

    fun getCorrectionFactor(): Float = m_correctionFactor
    fun setCorrectionFactor(correctionFactor: Float) { m_correctionFactor = correctionFactor }
    fun setLinearOffset(linearOffset: Vec2) {
        if (linearOffset != m_linearOffset) {
            getBodyA().setAwake(true)
            getBodyB().setAwake(true)
            m_linearOffset.set(linearOffset)
        }
    }
    fun getLinearOffset(out: Vec2) { out.set(m_linearOffset) }
    fun getLinearOffset(): Vec2 = m_linearOffset
    fun setAngularOffset(angularOffset: Float) {
        if (angularOffset != m_angularOffset) {
            getBodyA().setAwake(true)
            getBodyB().setAwake(true)
            m_angularOffset = angularOffset
        }
    }
    fun getAngularOffset(): Float = m_angularOffset
    fun setMaxForce(force: Float) { m_maxForce = force }
    fun getMaxForce(): Float = m_maxForce
    fun setMaxTorque(torque: Float) { m_maxTorque = torque }
    fun getMaxTorque(): Float = m_maxTorque

    override fun getAnchorA(out: Vec2) { out.set(getBodyA().getPosition()) }
    override fun getAnchorB(out: Vec2) { out.set(getBodyB().getPosition()) }
    override fun getReactionForce(invDt: Float, out: Vec2) { out.set(m_linearImpulse).mulLocal(invDt) }
    override fun getReactionTorque(invDt: Float): Float = m_angularImpulse * invDt

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
        val k = pool.popMat22()
        qA.set(aA)
        qB.set(aB)

        m_rA.x = qA.c * -m_localCenterA.x - qA.s * -m_localCenterA.y
        m_rA.y = qA.s * -m_localCenterA.x + qA.c * -m_localCenterA.y
        m_rB.x = qB.c * -m_localCenterB.x - qB.s * -m_localCenterB.y
        m_rB.y = qB.s * -m_localCenterB.x + qB.c * -m_localCenterB.y

        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB
        k.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y
        k.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y
        k.ey.x = k.ex.y
        k.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x
        k.invertToOut(m_linearMass)

        m_angularMass = iA + iB
        if (m_angularMass > 0f) {
            m_angularMass = 1f / m_angularMass
        }

        Rot.mulToOutUnsafe(qA, m_linearOffset, temp)
        m_linearError.x = cB.x + m_rB.x - cA.x - m_rA.x - temp.x
        m_linearError.y = cB.y + m_rB.y - cA.y - m_rA.y - temp.y
        m_angularError = aB - aA - m_angularOffset

        if (step.warmStarting) {
            m_linearImpulse.mulLocal(step.dtRatio)
            m_angularImpulse *= step.dtRatio
            val p = m_linearImpulse
            vA.x -= mA * p.x
            vA.y -= mA * p.y
            wA -= iA * (m_rA.x * p.y - m_rA.y * p.x + m_angularImpulse)
            vB.x += mB * p.x
            vB.y += mB * p.y
            wB += iB * (m_rB.x * p.y - m_rB.y * p.x + m_angularImpulse)
        } else {
            m_linearImpulse.setZero()
            m_angularImpulse = 0f
        }

        pool.pushVec2(1)
        pool.pushMat22(1)
        pool.pushRot(2)
        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
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
        val h = step.dt
        val invH = step.inv_dt
        val temp = pool.popVec2()

        run {
            val cDot = wB - wA + invH * m_correctionFactor * m_angularError
            var impulse = -m_angularMass * cDot
            val oldImpulse = m_angularImpulse
            val maxImpulse = h * m_maxTorque
            m_angularImpulse = MathUtils.clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse)
            impulse = m_angularImpulse - oldImpulse
            wA -= iA * impulse
            wB += iB * impulse
        }

        val cDot = pool.popVec2()
        run {
            cDot.x = vB.x - wB * m_rB.y - vA.x + wA * m_rA.y + invH * m_correctionFactor * m_linearError.x
            cDot.y = vB.y + wB * m_rB.x - vA.y - wA * m_rA.x + invH * m_correctionFactor * m_linearError.y
            val impulse = temp
            Mat22.mulToOutUnsafe(m_linearMass, cDot, impulse)
            impulse.negateLocal()
            val oldImpulse = pool.popVec2()
            oldImpulse.set(m_linearImpulse)
            m_linearImpulse.addLocal(impulse)
            val maxImpulse = h * m_maxForce
            if (m_linearImpulse.lengthSquared() > maxImpulse * maxImpulse) {
                m_linearImpulse.normalize()
                m_linearImpulse.mulLocal(maxImpulse)
            }
            impulse.x = m_linearImpulse.x - oldImpulse.x
            impulse.y = m_linearImpulse.y - oldImpulse.y
            vA.x -= mA * impulse.x
            vA.y -= mA * impulse.y
            wA -= iA * (m_rA.x * impulse.y - m_rA.y * impulse.x)
            vB.x += mB * impulse.x
            vB.y += mB * impulse.y
            wB += iB * (m_rB.x * impulse.y - m_rB.y * impulse.x)
        }

        pool.pushVec2(3)
        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
    }

    override fun solvePositionConstraints(data: SolverData): Boolean = true
}
