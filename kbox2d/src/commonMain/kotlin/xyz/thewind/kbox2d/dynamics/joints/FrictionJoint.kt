package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Mat22
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.pooling.IWorldPool

class FrictionJoint(
    pool: IWorldPool,
    def: FrictionJointDef,
) : Joint(pool, def) {
    private val m_localAnchorA: Vec2 = def.localAnchorA.clone()
    private val m_localAnchorB: Vec2 = def.localAnchorB.clone()
    private val m_linearImpulse: Vec2 = Vec2()
    private var m_angularImpulse: Float = 0f
    private var m_maxForce: Float = def.maxForce
    private var m_maxTorque: Float = def.maxTorque
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
    private val m_linearMass: Mat22 = Mat22()
    private var m_angularMass: Float = 0f

    fun getLocalAnchorA(): Vec2 = m_localAnchorA
    fun getLocalAnchorB(): Vec2 = m_localAnchorB
    fun setMaxForce(force: Float) { m_maxForce = force }
    fun getMaxForce(): Float = m_maxForce
    fun setMaxTorque(torque: Float) { m_maxTorque = torque }
    fun getMaxTorque(): Float = m_maxTorque

    override fun getAnchorA(out: Vec2) {
        getBodyA().getWorldPointToOut(m_localAnchorA, out)
    }

    override fun getAnchorB(out: Vec2) {
        getBodyB().getWorldPointToOut(m_localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.set(m_linearImpulse).mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float = invDt * m_angularImpulse

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

        val aA = data.positions[m_indexA].a
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val aB = data.positions[m_indexB].a
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w

        val temp = pool.popVec2()
        val qA = pool.popRot()
        val qB = pool.popRot()
        qA.set(aA)
        qB.set(aB)

        Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), m_rA)
        Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), m_rB)

        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB
        val k = pool.popMat22()
        k.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y
        k.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y
        k.ey.x = k.ex.y
        k.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x
        k.invertToOut(m_linearMass)

        m_angularMass = iA + iB
        if (m_angularMass > 0f) {
            m_angularMass = 1f / m_angularMass
        }

        if (step.warmStarting) {
            m_linearImpulse.mulLocal(step.dtRatio)
            m_angularImpulse *= step.dtRatio
            val p = pool.popVec2()
            p.set(m_linearImpulse)
            temp.set(p).mulLocal(mA)
            vA.subLocal(temp)
            wA -= iA * (Vec2.cross(m_rA, p) + m_angularImpulse)
            temp.set(p).mulLocal(mB)
            vB.addLocal(temp)
            wB += iB * (Vec2.cross(m_rB, p) + m_angularImpulse)
            pool.pushVec2(1)
        } else {
            m_linearImpulse.setZero()
            m_angularImpulse = 0f
        }

        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
        pool.pushRot(2)
        pool.pushVec2(1)
        pool.pushMat22(1)
    }

    override fun solveVelocityConstraints(data: SolverData) {
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w
        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB
        val h = data.step!!.dt

        run {
            val cDot = wB - wA
            var impulse = -m_angularMass * cDot
            val oldImpulse = m_angularImpulse
            val maxImpulse = h * m_maxTorque
            m_angularImpulse = MathUtils.clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse)
            impulse = m_angularImpulse - oldImpulse
            wA -= iA * impulse
            wB += iB * impulse
        }

        run {
            val cDot = pool.popVec2()
            val temp = pool.popVec2()
            Vec2.crossToOutUnsafe(wA, m_rA, temp)
            Vec2.crossToOutUnsafe(wB, m_rB, cDot)
            cDot.addLocal(vB).subLocal(vA).subLocal(temp)
            val impulse = pool.popVec2()
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
            impulse.set(m_linearImpulse).subLocal(oldImpulse)
            temp.set(impulse).mulLocal(mA)
            vA.subLocal(temp)
            wA -= iA * Vec2.cross(m_rA, impulse)
            temp.set(impulse).mulLocal(mB)
            vB.addLocal(temp)
            wB += iB * Vec2.cross(m_rB, impulse)
            pool.pushVec2(4)
        }

        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
    }

    override fun solvePositionConstraints(data: SolverData): Boolean = true
}
