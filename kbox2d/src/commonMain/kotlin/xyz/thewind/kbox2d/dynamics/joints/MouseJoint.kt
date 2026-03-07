package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Mat22
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.pooling.IWorldPool

class MouseJoint(
    pool: IWorldPool,
    def: MouseJointDef,
) : Joint(pool, def) {
    private val m_localAnchorB: Vec2 = Vec2()
    private val m_targetA: Vec2 = Vec2()
    private var m_frequencyHz: Float = def.frequencyHz
    private var m_dampingRatio: Float = def.dampingRatio
    private var m_beta: Float = 0f
    private val m_impulse: Vec2 = Vec2()
    private var m_maxForce: Float = def.maxForce
    private var m_gamma: Float = 0f
    private var m_indexB: Int = 0
    private val m_rB: Vec2 = Vec2()
    private val m_localCenterB: Vec2 = Vec2()
    private var m_invMassB: Float = 0f
    private var m_invIB: Float = 0f
    private val m_mass: Mat22 = Mat22()
    private val m_C: Vec2 = Vec2()

    init {
        require(def.target.isValid())
        require(def.maxForce >= 0f)
        require(def.frequencyHz >= 0f)
        require(def.dampingRatio >= 0f)
        m_targetA.set(def.target)
        Transform.mulTransToOutUnsafe(getBodyB().getTransform(), m_targetA, m_localAnchorB)
    }

    override fun getAnchorA(out: Vec2) {
        out.set(m_targetA)
    }

    override fun getAnchorB(out: Vec2) {
        getBodyB().getWorldPointToOut(m_localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.set(m_impulse).mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float = 0f

    fun setTarget(target: Vec2) {
        if (!getBodyB().isAwake()) {
            getBodyB().setAwake(true)
        }
        m_targetA.set(target)
    }

    fun getTarget(): Vec2 = m_targetA
    fun setMaxForce(force: Float) { m_maxForce = force }
    fun getMaxForce(): Float = m_maxForce
    fun setFrequency(hz: Float) { m_frequencyHz = hz }
    fun getFrequency(): Float = m_frequencyHz
    fun setDampingRatio(ratio: Float) { m_dampingRatio = ratio }
    fun getDampingRatio(): Float = m_dampingRatio

    override fun initVelocityConstraints(data: SolverData) {
        val step = data.step!!
        m_indexB = getBodyB().m_islandIndex
        m_localCenterB.set(getBodyB().m_sweep.localCenter)
        m_invMassB = getBodyB().m_invMass
        m_invIB = getBodyB().m_invI
        val cB = data.positions[m_indexB].c
        val aB = data.positions[m_indexB].a
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w
        val qB = pool.popRot()
        qB.set(aB)

        val mass = getBodyB().getMass()
        val omega = 2f * MathUtils.PI * m_frequencyHz
        val d = 2f * mass * m_dampingRatio * omega
        val k = mass * omega * omega
        val h = step.dt
        require(d + h * k > Settings.EPSILON)
        m_gamma = h * (d + h * k)
        if (m_gamma != 0f) {
            m_gamma = 1f / m_gamma
        }
        m_beta = h * k * m_gamma

        val temp = pool.popVec2()
        Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), m_rB)
        val kMat = pool.popMat22()
        kMat.ex.x = m_invMassB + m_invIB * m_rB.y * m_rB.y + m_gamma
        kMat.ex.y = -m_invIB * m_rB.x * m_rB.y
        kMat.ey.x = kMat.ex.y
        kMat.ey.y = m_invMassB + m_invIB * m_rB.x * m_rB.x + m_gamma
        kMat.invertToOut(m_mass)

        m_C.set(cB).addLocal(m_rB).subLocal(m_targetA)
        m_C.mulLocal(m_beta)
        wB *= 0.98f

        if (step.warmStarting) {
            m_impulse.mulLocal(step.dtRatio)
            vB.x += m_invMassB * m_impulse.x
            vB.y += m_invMassB * m_impulse.y
            wB += m_invIB * Vec2.cross(m_rB, m_impulse)
        } else {
            m_impulse.setZero()
        }

        data.velocities[m_indexB].w = wB
        pool.pushVec2(1)
        pool.pushMat22(1)
        pool.pushRot(1)
    }

    override fun solvePositionConstraints(data: SolverData): Boolean = true

    override fun solveVelocityConstraints(data: SolverData) {
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w
        val cDot = pool.popVec2()
        Vec2.crossToOutUnsafe(wB, m_rB, cDot)
        cDot.addLocal(vB)
        val impulse = pool.popVec2()
        val temp = pool.popVec2()
        temp.set(m_impulse).mulLocal(m_gamma).addLocal(m_C).addLocal(cDot).negateLocal()
        Mat22.mulToOutUnsafe(m_mass, temp, impulse)
        val oldImpulse = temp
        oldImpulse.set(m_impulse)
        m_impulse.addLocal(impulse)
        val maxImpulse = data.step!!.dt * m_maxForce
        if (m_impulse.lengthSquared() > maxImpulse * maxImpulse) {
            m_impulse.mulLocal(maxImpulse / m_impulse.length())
        }
        impulse.set(m_impulse).subLocal(oldImpulse)
        vB.x += m_invMassB * impulse.x
        vB.y += m_invMassB * impulse.y
        wB += m_invIB * Vec2.cross(m_rB, impulse)
        data.velocities[m_indexB].w = wB
        pool.pushVec2(3)
    }
}
