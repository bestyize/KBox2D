package xyz.thewind.kbox2d.dynamics.contacts

import xyz.thewind.kbox2d.callbacks.ContactListener
import xyz.thewind.kbox2d.collision.ContactID
import xyz.thewind.kbox2d.collision.Manifold
import xyz.thewind.kbox2d.collision.ManifoldPoint
import xyz.thewind.kbox2d.collision.WorldManifold
import xyz.thewind.kbox2d.collision.shapes.Shape
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.dynamics.Body
import xyz.thewind.kbox2d.dynamics.Fixture
import xyz.thewind.kbox2d.pooling.IWorldPool

abstract class Contact(protected val pool: IWorldPool) {
    var m_flags: Int = 0
    var m_prev: Contact? = null
    var m_next: Contact? = null
    var m_nodeA: ContactEdge = ContactEdge()
    var m_nodeB: ContactEdge = ContactEdge()
    var m_fixtureA: Fixture? = null
    var m_fixtureB: Fixture? = null
    var m_indexA: Int = 0
    var m_indexB: Int = 0
    val m_manifold: Manifold = Manifold()
    var m_toiCount: Float = 0f
    var m_toi: Float = 0f
    var m_friction: Float = 0f
    var m_restitution: Float = 0f
    var m_tangentSpeed: Float = 0f
    private val oldManifold = Manifold()

    open fun init(fA: Fixture, indexA: Int, fB: Fixture, indexB: Int) {
        m_flags = ENABLED_FLAG
        m_fixtureA = fA
        m_fixtureB = fB
        m_indexA = indexA
        m_indexB = indexB
        m_manifold.pointCount = 0
        m_prev = null
        m_next = null
        m_nodeA.contact = null
        m_nodeA.prev = null
        m_nodeA.next = null
        m_nodeA.other = null
        m_nodeB.contact = null
        m_nodeB.prev = null
        m_nodeB.next = null
        m_nodeB.other = null
        m_toiCount = 0f
        m_friction = mixFriction(fA.m_friction, fB.m_friction)
        m_restitution = mixRestitution(fA.m_restitution, fB.m_restitution)
        m_tangentSpeed = 0f
    }

    fun getManifold(): Manifold = m_manifold

    fun getWorldManifold(worldManifold: WorldManifold) {
        val bodyA = m_fixtureA!!.getBody()!!
        val bodyB = m_fixtureB!!.getBody()!!
        val shapeA = m_fixtureA!!.getShape()
        val shapeB = m_fixtureB!!.getShape()
        worldManifold.initialize(m_manifold, bodyA.getTransform(), shapeA.m_radius, bodyB.getTransform(), shapeB.m_radius)
    }

    fun isTouching(): Boolean = (m_flags and TOUCHING_FLAG) == TOUCHING_FLAG

    fun setEnabled(flag: Boolean) {
        m_flags = if (flag) m_flags or ENABLED_FLAG else m_flags and ENABLED_FLAG.inv()
    }

    fun isEnabled(): Boolean = (m_flags and ENABLED_FLAG) == ENABLED_FLAG

    fun getNext(): Contact? = m_next
    fun getFixtureA(): Fixture = m_fixtureA!!
    fun getChildIndexA(): Int = m_indexA
    fun getFixtureB(): Fixture = m_fixtureB!!
    fun getChildIndexB(): Int = m_indexB
    fun setFriction(friction: Float) { m_friction = friction }
    fun getFriction(): Float = m_friction
    fun resetFriction() { m_friction = mixFriction(m_fixtureA!!.m_friction, m_fixtureB!!.m_friction) }
    fun setRestitution(restitution: Float) { m_restitution = restitution }
    fun getRestitution(): Float = m_restitution
    fun resetRestitution() { m_restitution = mixRestitution(m_fixtureA!!.m_restitution, m_fixtureB!!.m_restitution) }
    fun setTangentSpeed(speed: Float) { m_tangentSpeed = speed }
    fun getTangentSpeed(): Float = m_tangentSpeed

    abstract fun evaluate(manifold: Manifold, xfA: Transform, xfB: Transform)

    fun flagForFiltering() {
        m_flags = m_flags or FILTER_FLAG
    }

    fun update(listener: ContactListener?) {
        oldManifold.set(m_manifold)
        m_flags = m_flags or ENABLED_FLAG
        var touching = false
        val wasTouching = (m_flags and TOUCHING_FLAG) == TOUCHING_FLAG
        val sensorA = m_fixtureA!!.isSensor()
        val sensorB = m_fixtureB!!.isSensor()
        val sensor = sensorA || sensorB
        val bodyA = m_fixtureA!!.getBody()!!
        val bodyB = m_fixtureB!!.getBody()!!
        val xfA = bodyA.getTransform()
        val xfB = bodyB.getTransform()
        if (sensor) {
            val shapeA: Shape = m_fixtureA!!.getShape()
            val shapeB: Shape = m_fixtureB!!.getShape()
            touching = pool.getCollision().testOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB)
            m_manifold.pointCount = 0
        } else {
            evaluate(m_manifold, xfA, xfB)
            touching = m_manifold.pointCount > 0
            for (i in 0 until m_manifold.pointCount) {
                val mp2 = m_manifold.points[i]
                mp2.normalImpulse = 0f
                mp2.tangentImpulse = 0f
                val id2 = mp2.id
                for (j in 0 until oldManifold.pointCount) {
                    val mp1: ManifoldPoint = oldManifold.points[j]
                    if (mp1.id.isEqual(id2)) {
                        mp2.normalImpulse = mp1.normalImpulse
                        mp2.tangentImpulse = mp1.tangentImpulse
                        break
                    }
                }
            }
            if (touching != wasTouching) {
                bodyA.setAwake(true)
                bodyB.setAwake(true)
            }
        }
        if (touching) {
            m_flags = m_flags or TOUCHING_FLAG
        } else {
            m_flags = m_flags and TOUCHING_FLAG.inv()
        }
        if (listener == null) return
        if (!wasTouching && touching) listener.beginContact(this)
        if (wasTouching && !touching) listener.endContact(this)
        if (!sensor && touching) listener.preSolve(this, oldManifold)
    }

    companion object {
        const val ISLAND_FLAG = 0x0001
        const val TOUCHING_FLAG = 0x0002
        const val ENABLED_FLAG = 0x0004
        const val FILTER_FLAG = 0x0008
        const val BULLET_HIT_FLAG = 0x0010
        const val TOI_FLAG = 0x0020

        fun mixFriction(friction1: Float, friction2: Float): Float = MathUtils.sqrt(friction1 * friction2)
        fun mixRestitution(restitution1: Float, restitution2: Float): Float = if (restitution1 > restitution2) restitution1 else restitution2
    }
}
