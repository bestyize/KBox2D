package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Mat33
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.common.Vec3
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.pooling.IWorldPool

class WeldJoint(
    pool: IWorldPool,
    def: WeldJointDef,
) : Joint(pool, def) {
    private var m_frequencyHz: Float = def.frequencyHz
    private var m_dampingRatio: Float = def.dampingRatio
    private var m_bias: Float = 0f
    private val m_localAnchorA: Vec2 = def.localAnchorA.clone()
    private val m_localAnchorB: Vec2 = def.localAnchorB.clone()
    private var m_referenceAngle: Float = def.referenceAngle
    private var m_gamma: Float = 0f
    private val m_impulse: Vec3 = Vec3()
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

    fun getReferenceAngle(): Float = m_referenceAngle
    fun getLocalAnchorA(): Vec2 = m_localAnchorA
    fun getLocalAnchorB(): Vec2 = m_localAnchorB
    fun getFrequency(): Float = m_frequencyHz
    fun setFrequency(frequencyHz: Float) { m_frequencyHz = frequencyHz }
    fun getDampingRatio(): Float = m_dampingRatio
    fun setDampingRatio(dampingRatio: Float) { m_dampingRatio = dampingRatio }

    override fun getAnchorA(out: Vec2) {
        getBodyA().getWorldPointToOut(m_localAnchorA, out)
    }

    override fun getAnchorB(out: Vec2) {
        getBodyB().getWorldPointToOut(m_localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.set(m_impulse.x, m_impulse.y)
        out.mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float = invDt * m_impulse.z

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
        val k = pool.popMat33()
        k.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB
        k.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB
        k.ez.x = -m_rA.y * iA - m_rB.y * iB
        k.ex.y = k.ey.x
        k.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB
        k.ez.y = m_rA.x * iA + m_rB.x * iB
        k.ex.z = k.ez.x
        k.ey.z = k.ez.y
        k.ez.z = iA + iB

        if (m_frequencyHz > 0f) {
            k.getInverse22(m_mass)
            var invM = iA + iB
            val m = if (invM > 0f) 1f / invM else 0f
            val c = aB - aA - m_referenceAngle
            val omega = 2f * MathUtils.PI * m_frequencyHz
            val d = 2f * m * m_dampingRatio * omega
            val stiffness = m * omega * omega
            val h = step.dt
            m_gamma = h * (d + h * stiffness)
            m_gamma = if (m_gamma != 0f) 1f / m_gamma else 0f
            m_bias = c * h * stiffness * m_gamma
            invM += m_gamma
            m_mass.ez.z = if (invM != 0f) 1f / invM else 0f
        } else {
            k.getSymInverse33(m_mass)
            m_gamma = 0f
            m_bias = 0f
        }

        if (step.warmStarting) {
            val p = pool.popVec2()
            m_impulse.mulLocal(step.dtRatio)
            p.set(m_impulse.x, m_impulse.y)
            vA.x -= mA * p.x
            vA.y -= mA * p.y
            wA -= iA * (Vec2.cross(m_rA, p) + m_impulse.z)
            vB.x += mB * p.x
            vB.y += mB * p.y
            wB += iB * (Vec2.cross(m_rB, p) + m_impulse.z)
            pool.pushVec2(1)
        } else {
            m_impulse.setZero()
        }

        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
        pool.pushVec2(1)
        pool.pushRot(2)
        pool.pushMat33(1)
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
        val cDot1 = pool.popVec2()
        val p = pool.popVec2()
        val temp = pool.popVec2()

        if (m_frequencyHz > 0f) {
            val cDot2 = wB - wA
            val impulse2 = -m_mass.ez.z * (cDot2 + m_bias + m_gamma * m_impulse.z)
            m_impulse.z += impulse2
            wA -= iA * impulse2
            wB += iB * impulse2

            Vec2.crossToOutUnsafe(wB, m_rB, cDot1)
            Vec2.crossToOutUnsafe(wA, m_rA, temp)
            cDot1.addLocal(vB).subLocal(vA).subLocal(temp)
            val impulse1 = p
            Mat33.mul22ToOutUnsafe(m_mass, cDot1, impulse1)
            impulse1.negateLocal()
            m_impulse.x += impulse1.x
            m_impulse.y += impulse1.y
            vA.x -= mA * p.x
            vA.y -= mA * p.y
            wA -= iA * Vec2.cross(m_rA, p)
            vB.x += mB * p.x
            vB.y += mB * p.y
            wB += iB * Vec2.cross(m_rB, p)
        } else {
            Vec2.crossToOutUnsafe(wA, m_rA, temp)
            Vec2.crossToOutUnsafe(wB, m_rB, cDot1)
            cDot1.addLocal(vB).subLocal(vA).subLocal(temp)
            val cDot2 = wB - wA
            val cDot = pool.popVec3()
            cDot.set(cDot1.x, cDot1.y, cDot2)
            val impulse = pool.popVec3()
            Mat33.mulToOutUnsafe(m_mass, cDot, impulse)
            impulse.negateLocal()
            m_impulse.addLocal(impulse)
            p.set(impulse.x, impulse.y)
            vA.x -= mA * p.x
            vA.y -= mA * p.y
            wA -= iA * (Vec2.cross(m_rA, p) + impulse.z)
            vB.x += mB * p.x
            vB.y += mB * p.y
            wB += iB * (Vec2.cross(m_rB, p) + impulse.z)
            pool.pushVec3(2)
        }

        data.velocities[m_indexA].w = wA
        data.velocities[m_indexB].w = wB
        pool.pushVec2(3)
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        val cA = data.positions[m_indexA].c
        var aA = data.positions[m_indexA].a
        val cB = data.positions[m_indexB].c
        var aB = data.positions[m_indexB].a
        val qA = pool.popRot()
        val qB = pool.popRot()
        val temp = pool.popVec2()
        val rA = pool.popVec2()
        val rB = pool.popVec2()
        qA.set(aA)
        qB.set(aB)
        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB
        Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), rA)
        Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), rB)

        val k = pool.popMat33()
        val c1 = pool.popVec2()
        val p = pool.popVec2()
        k.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB
        k.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB
        k.ez.x = -rA.y * iA - rB.y * iB
        k.ex.y = k.ey.x
        k.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB
        k.ez.y = rA.x * iA + rB.x * iB
        k.ex.z = k.ez.x
        k.ey.z = k.ez.y
        k.ez.z = iA + iB

        val positionError: Float
        val angularError: Float
        if (m_frequencyHz > 0f) {
            c1.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)
            positionError = c1.length()
            angularError = 0f
            k.solve22ToOut(c1, p)
            p.negateLocal()
            cA.x -= mA * p.x
            cA.y -= mA * p.y
            aA -= iA * Vec2.cross(rA, p)
            cB.x += mB * p.x
            cB.y += mB * p.y
            aB += iB * Vec2.cross(rB, p)
        } else {
            c1.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)
            val c2 = aB - aA - m_referenceAngle
            positionError = c1.length()
            angularError = MathUtils.abs(c2)
            val c = pool.popVec3()
            val impulse = pool.popVec3()
            c.set(c1.x, c1.y, c2)
            k.solve33ToOut(c, impulse)
            impulse.negateLocal()
            p.set(impulse.x, impulse.y)
            cA.x -= mA * p.x
            cA.y -= mA * p.y
            aA -= iA * (Vec2.cross(rA, p) + impulse.z)
            cB.x += mB * p.x
            cB.y += mB * p.y
            aB += iB * (Vec2.cross(rB, p) + impulse.z)
            pool.pushVec3(2)
        }

        data.positions[m_indexA].a = aA
        data.positions[m_indexB].a = aB
        pool.pushVec2(5)
        pool.pushRot(2)
        pool.pushMat33(1)
        return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop
    }
}
