package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body
import xyz.thewind.kbox2d.dynamics.SolverData
import xyz.thewind.kbox2d.dynamics.World

class ConstantVolumeJoint(
    private val world: World,
    def: ConstantVolumeJointDef,
) : Joint(world.getPool(), def) {
    private val bodies: Array<Body>
    private val targetLengths: FloatArray
    private var targetVolume: Float
    private val normals: Array<Vec2>
    private var m_impulse: Float = 0f
    private val distanceJoints: Array<DistanceJoint>

    init {
        require(def.bodies.size > 2) { "You cannot create a constant volume joint with less than three bodies." }
        bodies = def.bodies.toTypedArray()
        targetLengths = FloatArray(bodies.size)
        for (i in targetLengths.indices) {
            val next = if (i == targetLengths.lastIndex) 0 else i + 1
            targetLengths[i] = bodies[i].getWorldCenter().sub(bodies[next].getWorldCenter()).length()
        }
        targetVolume = getBodyArea()
        require(def.joints.isEmpty() || def.joints.size == def.bodies.size) {
            "Incorrect joint definition. Joints have to correspond to the bodies"
        }
        distanceJoints = if (def.joints.isEmpty()) {
            Array(bodies.size) { i ->
                val next = if (i == targetLengths.lastIndex) 0 else i + 1
                DistanceJointDef().apply {
                    frequencyHz = def.frequencyHz
                    dampingRatio = def.dampingRatio
                    collideConnected = def.collideConnected
                    initialize(bodies[i], bodies[next], bodies[i].getWorldCenter(), bodies[next].getWorldCenter())
                }.let { world.createJoint(it) as DistanceJoint }
            }
        } else {
            Array(def.joints.size) { index -> def.joints[index] as DistanceJoint }
        }
        normals = Array(bodies.size) { Vec2() }
    }

    fun getBodies(): Array<Body> = bodies

    fun getJoints(): Array<DistanceJoint> = distanceJoints

    fun inflate(factor: Float) {
        targetVolume *= factor
    }

    override fun destructor() {
        for (joint in distanceJoints) {
            world.destroyJoint(joint)
        }
    }

    private fun getBodyArea(): Float {
        var area = 0f
        for (i in bodies.indices) {
            val next = if (i == bodies.lastIndex) 0 else i + 1
            area += bodies[i].getWorldCenter().x * bodies[next].getWorldCenter().y - bodies[next].getWorldCenter().x * bodies[i].getWorldCenter().y
        }
        return area * 0.5f
    }

    private fun getSolverArea(positions: Array<xyz.thewind.kbox2d.dynamics.contacts.Position>): Float {
        var area = 0f
        for (i in bodies.indices) {
            val next = if (i == bodies.lastIndex) 0 else i + 1
            area += positions[bodies[i].m_islandIndex].c.x * positions[bodies[next].m_islandIndex].c.y -
                positions[bodies[next].m_islandIndex].c.x * positions[bodies[i].m_islandIndex].c.y
        }
        return area * 0.5f
    }

    private fun constrainEdges(positions: Array<xyz.thewind.kbox2d.dynamics.contacts.Position>): Boolean {
        var perimeter = 0f
        for (i in bodies.indices) {
            val next = if (i == bodies.lastIndex) 0 else i + 1
            val dx = positions[bodies[next].m_islandIndex].c.x - positions[bodies[i].m_islandIndex].c.x
            val dy = positions[bodies[next].m_islandIndex].c.y - positions[bodies[i].m_islandIndex].c.y
            var dist = MathUtils.sqrt(dx * dx + dy * dy)
            if (dist < Settings.EPSILON) {
                dist = 1f
            }
            normals[i].x = dy / dist
            normals[i].y = -dx / dist
            perimeter += dist
        }

        val delta = pool.popVec2()
        val deltaArea = targetVolume - getSolverArea(positions)
        val toExtrude = 0.5f * deltaArea / perimeter
        var done = true
        for (i in bodies.indices) {
            val next = if (i == bodies.lastIndex) 0 else i + 1
            delta.set(
                toExtrude * (normals[i].x + normals[next].x),
                toExtrude * (normals[i].y + normals[next].y),
            )
            val normSq = delta.lengthSquared()
            if (normSq > Settings.maxLinearCorrection * Settings.maxLinearCorrection) {
                delta.mulLocal(Settings.maxLinearCorrection / MathUtils.sqrt(normSq))
            }
            if (normSq > Settings.linearSlop * Settings.linearSlop) {
                done = false
            }
            positions[bodies[next].m_islandIndex].c.x += delta.x
            positions[bodies[next].m_islandIndex].c.y += delta.y
        }
        pool.pushVec2(1)
        return done
    }

    override fun initVelocityConstraints(data: SolverData) {
        val velocities = data.velocities
        val positions = data.positions
        val d = pool.getVec2Array(bodies.size)
        for (i in bodies.indices) {
            val prev = if (i == 0) bodies.lastIndex else i - 1
            val next = if (i == bodies.lastIndex) 0 else i + 1
            d[i].set(positions[bodies[next].m_islandIndex].c).subLocal(positions[bodies[prev].m_islandIndex].c)
        }
        if (data.step!!.warmStarting) {
            m_impulse *= data.step!!.dtRatio
            for (i in bodies.indices) {
                velocities[bodies[i].m_islandIndex].v.x += bodies[i].m_invMass * d[i].y * 0.5f * m_impulse
                velocities[bodies[i].m_islandIndex].v.y += bodies[i].m_invMass * -d[i].x * 0.5f * m_impulse
            }
        } else {
            m_impulse = 0f
        }
    }

    override fun solvePositionConstraints(data: SolverData): Boolean = constrainEdges(data.positions)

    override fun solveVelocityConstraints(data: SolverData) {
        var crossMassSum = 0f
        var dotMassSum = 0f
        val velocities = data.velocities
        val positions = data.positions
        val d = pool.getVec2Array(bodies.size)
        for (i in bodies.indices) {
            val prev = if (i == 0) bodies.lastIndex else i - 1
            val next = if (i == bodies.lastIndex) 0 else i + 1
            d[i].set(positions[bodies[next].m_islandIndex].c).subLocal(positions[bodies[prev].m_islandIndex].c)
            dotMassSum += d[i].lengthSquared() / bodies[i].getMass()
            crossMassSum += Vec2.cross(velocities[bodies[i].m_islandIndex].v, d[i])
        }
        val lambda = -2f * crossMassSum / dotMassSum
        m_impulse += lambda
        for (i in bodies.indices) {
            velocities[bodies[i].m_islandIndex].v.x += bodies[i].m_invMass * d[i].y * 0.5f * lambda
            velocities[bodies[i].m_islandIndex].v.y += bodies[i].m_invMass * -d[i].x * 0.5f * lambda
        }
    }

    override fun getAnchorA(out: Vec2) {
        out.setZero()
    }

    override fun getAnchorB(out: Vec2) {
        out.setZero()
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.setZero()
    }

    override fun getReactionTorque(invDt: Float): Float = 0f
}
