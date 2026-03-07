package xyz.thewind.kbox2d.dynamics

import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.RayCastInput
import xyz.thewind.kbox2d.collision.RayCastOutput
import xyz.thewind.kbox2d.collision.broadphase.BroadPhase
import xyz.thewind.kbox2d.collision.shapes.MassData
import xyz.thewind.kbox2d.collision.shapes.Shape
import xyz.thewind.kbox2d.collision.shapes.ShapeType
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.contacts.Contact
import xyz.thewind.kbox2d.dynamics.contacts.ContactEdge

open class Fixture {
    var m_density: Float = 0f
    var m_next: Fixture? = null
    var m_body: Body? = null
    var m_shape: Shape? = null
    var m_friction: Float = 0f
    var m_restitution: Float = 0f
    var m_proxies: Array<FixtureProxy>? = null
    var m_proxyCount: Int = 0
    val m_filter: Filter = Filter()
    var m_isSensor: Boolean = false
    var m_userData: Any? = null

    private val pool1 = AABB()
    private val pool2 = AABB()
    private val displacement = Vec2()

    fun getType(): ShapeType = m_shape!!.getType()

    fun getShape(): Shape = m_shape!!

    fun isSensor(): Boolean = m_isSensor

    fun setSensor(sensor: Boolean) {
        if (sensor != m_isSensor) {
            m_body?.setAwake(true)
            m_isSensor = sensor
        }
    }

    fun setFilterData(filter: Filter) {
        m_filter.set(filter)
        refilter()
    }

    fun getFilterData(): Filter = m_filter

    fun refilter() {
        val body = m_body ?: return
        var edge: ContactEdge? = body.getContactList()
        while (edge != null) {
            val contact: Contact? = edge.contact
            val fixtureA = contact?.getFixtureA()
            val fixtureB = contact?.getFixtureB()
            if (fixtureA === this || fixtureB === this) {
                contact.flagForFiltering()
            }
            edge = edge.next
        }
        val world = body.getWorld() ?: return
        val broadPhase = world.m_contactManager.m_broadPhase
        for (index in 0 until m_proxyCount) {
            broadPhase.touchProxy(m_proxies!![index].proxyId)
        }
    }

    fun getBody(): Body? = m_body

    fun getNext(): Fixture? = m_next

    fun setDensity(density: Float) {
        assert(density >= 0f)
        m_density = density
    }

    fun getDensity(): Float = m_density

    fun getUserData(): Any? = m_userData

    fun setUserData(data: Any?) {
        m_userData = data
    }

    fun testPoint(p: Vec2): Boolean = m_shape!!.testPoint(m_body!!.m_xf, p)

    fun raycast(output: RayCastOutput, input: RayCastInput, childIndex: Int): Boolean =
        m_shape!!.raycast(output, input, m_body!!.m_xf, childIndex)

    fun getMassData(massData: MassData) {
        m_shape!!.computeMass(massData, m_density)
    }

    fun getFriction(): Float = m_friction

    fun setFriction(friction: Float) {
        m_friction = friction
    }

    fun getRestitution(): Float = m_restitution

    fun setRestitution(restitution: Float) {
        m_restitution = restitution
    }

    fun getAABB(childIndex: Int): AABB {
        assert(childIndex >= 0 && childIndex < m_proxyCount)
        return m_proxies!![childIndex].aabb
    }

    fun computeDistance(p: Vec2, childIndex: Int, normalOut: Vec2): Float =
        m_shape!!.computeDistanceToOut(m_body!!.getTransform(), p, childIndex, normalOut)

    fun create(body: Body, def: FixtureDef) {
        m_userData = def.userData
        m_friction = def.friction
        m_restitution = def.restitution
        m_body = body
        m_next = null
        m_filter.set(def.filter)
        m_isSensor = def.isSensor
        m_shape = def.shape!!.clone()
        val childCount = m_shape!!.getChildCount()
        if (m_proxies == null) {
            m_proxies = Array(childCount) { FixtureProxy().also { proxy ->
                proxy.fixture = null
                proxy.proxyId = BroadPhase.NULL_PROXY
            } }
        }
        if (m_proxies!!.size < childCount) {
            val old = m_proxies!!
            val newLen = MathUtils.max(old.size * 2, childCount)
            m_proxies = Array(newLen) { index ->
                if (index < old.size) old[index] else FixtureProxy()
            }
            for (proxy in m_proxies!!) {
                proxy.fixture = null
                if (proxy.proxyId != BroadPhase.NULL_PROXY) proxy.proxyId = BroadPhase.NULL_PROXY
            }
        }
        m_proxyCount = 0
        m_density = def.density
    }

    fun destroy() {
        assert(m_proxyCount == 0)
        m_shape = null
        m_proxies = null
        m_next = null
    }

    fun createProxies(broadPhase: BroadPhase, xf: Transform) {
        assert(m_proxyCount == 0)
        m_proxyCount = m_shape!!.getChildCount()
        for (index in 0 until m_proxyCount) {
            val proxy = m_proxies!![index]
            m_shape!!.computeAABB(proxy.aabb, xf, index)
            proxy.proxyId = broadPhase.createProxy(proxy.aabb, proxy)
            proxy.fixture = this
            proxy.childIndex = index
        }
    }

    fun destroyProxies(broadPhase: BroadPhase) {
        for (index in 0 until m_proxyCount) {
            val proxy = m_proxies!![index]
            broadPhase.destroyProxy(proxy.proxyId)
            proxy.proxyId = BroadPhase.NULL_PROXY
        }
        m_proxyCount = 0
    }

    internal fun synchronize(broadPhase: BroadPhase, transform1: Transform, transform2: Transform) {
        if (m_proxyCount == 0) return
        for (index in 0 until m_proxyCount) {
            val proxy = m_proxies!![index]
            val aabb1 = pool1
            val aab = pool2
            m_shape!!.computeAABB(aabb1, transform1, proxy.childIndex)
            m_shape!!.computeAABB(aab, transform2, proxy.childIndex)
            proxy.aabb.lowerBound.x = if (aabb1.lowerBound.x < aab.lowerBound.x) aabb1.lowerBound.x else aab.lowerBound.x
            proxy.aabb.lowerBound.y = if (aabb1.lowerBound.y < aab.lowerBound.y) aabb1.lowerBound.y else aab.lowerBound.y
            proxy.aabb.upperBound.x = if (aabb1.upperBound.x > aab.upperBound.x) aabb1.upperBound.x else aab.upperBound.x
            proxy.aabb.upperBound.y = if (aabb1.upperBound.y > aab.upperBound.y) aabb1.upperBound.y else aab.upperBound.y
            displacement.x = transform2.p.x - transform1.p.x
            displacement.y = transform2.p.y - transform1.p.y
            broadPhase.moveProxy(proxy.proxyId, proxy.aabb, displacement)
        }
    }
}
