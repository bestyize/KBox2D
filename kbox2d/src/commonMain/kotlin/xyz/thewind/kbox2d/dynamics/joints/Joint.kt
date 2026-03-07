package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.dynamics.World
import xyz.thewind.kbox2d.pooling.IWorldPool

abstract class Joint(
    protected val pool: IWorldPool,
    def: JointDef,
) {
    private val m_type: JointType = def.type
    var m_prev: Joint? = null
    var m_next: Joint? = null
    val m_edgeA: JointEdge = JointEdge()
    val m_edgeB: JointEdge = JointEdge()
    protected val m_bodyA: Body = requireNotNull(def.bodyA)
    protected val m_bodyB: Body = requireNotNull(def.bodyB)
    var m_islandFlag: Boolean = false
    private val m_collideConnected: Boolean = def.collideConnected
    var m_userData: Any? = def.userData

    fun getType(): JointType = m_type
    fun getBodyA(): Body = m_bodyA
    fun getBodyB(): Body = m_bodyB
    fun getNext(): Joint? = m_next
    fun getUserData(): Any? = m_userData
    fun setUserData(data: Any?) { m_userData = data }
    fun getCollideConnected(): Boolean = m_collideConnected
    fun isActive(): Boolean = m_bodyA.isActive() && m_bodyB.isActive()

    abstract fun getAnchorA(out: Vec2)
    abstract fun getAnchorB(out: Vec2)
    abstract fun getReactionForce(invDt: Float, out: Vec2)
    abstract fun getReactionTorque(invDt: Float): Float
    abstract fun initVelocityConstraints(data: SolverData)
    abstract fun solveVelocityConstraints(data: SolverData)
    abstract fun solvePositionConstraints(data: SolverData): Boolean

    open fun destructor() = Unit

    companion object {
        fun create(world: World, def: JointDef): Joint = when (def.type) {
            JointType.DISTANCE -> DistanceJoint(world.getPool(), def as DistanceJointDef)
            JointType.FRICTION -> FrictionJoint(world.getPool(), def as FrictionJointDef)
            JointType.MOTOR -> MotorJoint(world.getPool(), def as MotorJointDef)
            JointType.MOUSE -> MouseJoint(world.getPool(), def as MouseJointDef)
            JointType.ROPE -> RopeJoint(world.getPool(), def as RopeJointDef)
            JointType.WELD -> WeldJoint(world.getPool(), def as WeldJointDef)
            JointType.REVOLUTE -> RevoluteJoint(world.getPool(), def as RevoluteJointDef)
            JointType.PRISMATIC -> PrismaticJoint(world.getPool(), def as PrismaticJointDef)
            JointType.WHEEL -> WheelJoint(world.getPool(), def as WheelJointDef)
            JointType.PULLEY -> PulleyJoint(world.getPool(), def as PulleyJointDef)
            JointType.GEAR -> (def as GearJointDef).also {
                if (it.bodyA == null) it.bodyA = it.joint1?.getBodyB()
                if (it.bodyB == null) it.bodyB = it.joint2?.getBodyB()
            }.let { GearJoint(world.getPool(), it) }
            JointType.CONSTANT_VOLUME -> (def as ConstantVolumeJointDef).also {
                if (it.bodyA == null && it.bodies.isNotEmpty()) it.bodyA = it.bodies[0]
                if (it.bodyB == null && it.bodies.size > 1) it.bodyB = it.bodies[1]
            }.let { ConstantVolumeJoint(world, it) }
            JointType.UNKNOWN -> UnsupportedJoint(world.getPool(), def)
        }

        fun destroy(joint: Joint) {
            joint.destructor()
        }
    }
}

open class UnsupportedJoint(
    pool: IWorldPool,
    def: JointDef,
) : Joint(pool, def) {
    override fun getAnchorA(out: Vec2) {
        getBodyA().getWorldCenter().let(out::set)
    }

    override fun getAnchorB(out: Vec2) {
        getBodyB().getWorldCenter().let(out::set)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.setZero()
    }

    override fun getReactionTorque(invDt: Float): Float = 0f
    override fun initVelocityConstraints(data: SolverData) = Unit
    override fun solveVelocityConstraints(data: SolverData) = Unit
    override fun solvePositionConstraints(data: SolverData): Boolean = true
}
