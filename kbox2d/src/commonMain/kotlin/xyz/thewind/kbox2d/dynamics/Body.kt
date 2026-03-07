package xyz.thewind.kbox2d.dynamics

import xyz.thewind.kbox2d.collision.shapes.MassData
import xyz.thewind.kbox2d.collision.shapes.Shape
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Sweep
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.contacts.Contact
import xyz.thewind.kbox2d.dynamics.contacts.ContactEdge
import xyz.thewind.kbox2d.dynamics.joints.JointEdge

open class Body(
    bodyDef: BodyDef = BodyDef(),
    var m_world: World? = null,
) {
    val m_xf: Transform = Transform()
    val m_xf0: Transform = Transform()
    val m_sweep: Sweep = Sweep()
    val m_linearVelocity: Vec2 = Vec2()
    var m_angularVelocity: Float = 0f
    val m_force: Vec2 = Vec2()
    var m_torque: Float = 0f
    var m_prev: Body? = null
    var m_next: Body? = null
    var m_islandIndex: Int = 0
    var m_jointList: JointEdge? = null
    var m_contactList: ContactEdge? = null
    var m_fixtureList: Fixture? = null
    var m_fixtureCount: Int = 0
    var m_type: BodyType = bodyDef.type
    var m_mass: Float = 0f
    var m_invMass: Float = 0f
    var m_I: Float = 0f
    var m_invI: Float = 0f
    var m_linearDamping: Float = 0f
    var m_angularDamping: Float = 0f
    var m_gravityScale: Float = 1f
    var m_sleepTime: Float = 0f
    var m_userData: Any? = null
    var m_flags: Int = 0

    private val fixDef = FixtureDef()
    private val massData = MassData()
    private val oldCenter = Vec2()
    private val localCenter = Vec2()
    private val temp = Vec2()

    init {
        require(bodyDef.position.isValid())
        require(bodyDef.linearVelocity.isValid())
        require(bodyDef.gravityScale >= 0f)
        require(bodyDef.angularDamping >= 0f)
        require(bodyDef.linearDamping >= 0f)

        if (bodyDef.bullet) {
            m_flags = m_flags or e_bulletFlag
        }
        if (bodyDef.fixedRotation) {
            m_flags = m_flags or e_fixedRotationFlag
        }
        if (bodyDef.allowSleep) {
            m_flags = m_flags or e_autoSleepFlag
        }
        if (bodyDef.awake) {
            m_flags = m_flags or e_awakeFlag
        }
        if (bodyDef.active) {
            m_flags = m_flags or e_activeFlag
        }

        m_xf.p.set(bodyDef.position)
        m_xf.q.set(bodyDef.angle)
        m_xf0.set(m_xf)

        m_sweep.localCenter.setZero()
        m_sweep.c0.set(m_xf.p)
        m_sweep.c.set(m_xf.p)
        m_sweep.a0 = bodyDef.angle
        m_sweep.a = bodyDef.angle
        m_sweep.alpha0 = 0f

        m_linearVelocity.set(bodyDef.linearVelocity)
        m_angularVelocity = bodyDef.angularVelocity
        m_linearDamping = bodyDef.linearDamping
        m_angularDamping = bodyDef.angularDamping
        m_gravityScale = bodyDef.gravityScale
        m_force.setZero()
        m_torque = 0f
        m_sleepTime = 0f
        m_userData = bodyDef.userData

        if (m_type == BodyType.DYNAMIC) {
            m_mass = 1f
            m_invMass = 1f
        }
    }

    constructor(world: World? = null, type: BodyType = BodyType.STATIC) : this(
        bodyDef = BodyDef().apply { this.type = type },
        m_world = world,
    )

    fun getTransform(): Transform = m_xf

    fun getPosition(): Vec2 = m_xf.p

    fun getAngle(): Float = m_sweep.a

    fun getWorldCenter(): Vec2 = m_sweep.c

    fun getLocalCenter(): Vec2 = m_sweep.localCenter

    fun getWorld(): World? = m_world

    fun getContactList(): ContactEdge? = m_contactList

    fun getFixtureList(): Fixture? = m_fixtureList

    fun getNext(): Body? = m_next

    fun getType(): BodyType = m_type

    fun getMass(): Float = m_mass

    fun getInertia(): Float = m_I + m_mass * Vec2.dot(m_sweep.localCenter, m_sweep.localCenter)

    internal fun synchronizeTransform() {
        m_xf.q.set(m_sweep.a)
        val q = m_xf.q
        m_xf.p.x = m_sweep.c.x - (q.c * m_sweep.localCenter.x - q.s * m_sweep.localCenter.y)
        m_xf.p.y = m_sweep.c.y - (q.s * m_sweep.localCenter.x + q.c * m_sweep.localCenter.y)
    }

    internal fun synchronizeFixtures() {
        val broadPhase = m_world?.m_contactManager?.m_broadPhase ?: return
        m_xf0.q.set(m_sweep.a0)
        val q = m_xf0.q
        m_xf0.p.x = m_sweep.c0.x - (q.c * m_sweep.localCenter.x - q.s * m_sweep.localCenter.y)
        m_xf0.p.y = m_sweep.c0.y - (q.s * m_sweep.localCenter.x + q.c * m_sweep.localCenter.y)
        var fixture = m_fixtureList
        while (fixture != null) {
            fixture.synchronize(broadPhase, m_xf0, m_xf)
            fixture = fixture.m_next
        }
    }

    internal fun advance(alpha: Float) {
        m_sweep.advance(alpha)
        m_sweep.c.set(m_sweep.c0)
        m_sweep.a = m_sweep.a0
        m_xf.q.set(m_sweep.a)
        Rot.mulToOutUnsafe(m_xf.q, m_sweep.localCenter, m_xf.p)
        m_xf.p.mulLocal(-1f).addLocal(m_sweep.c)
    }

    fun setTransform(position: Vec2, angle: Float) {
        val world = m_world
        if (world != null && world.isLocked()) {
            return
        }
        m_xf.q.set(angle)
        m_xf.p.set(position)
        m_xf0.set(m_xf)
        Transform.mulToOutUnsafe(m_xf, m_sweep.localCenter, m_sweep.c)
        m_sweep.a = angle
        m_sweep.c0.set(m_sweep.c)
        m_sweep.a0 = m_sweep.a
        val broadPhase = world?.m_contactManager?.m_broadPhase ?: return
        var fixture = m_fixtureList
        while (fixture != null) {
            fixture.synchronize(broadPhase, m_xf, m_xf)
            fixture = fixture.m_next
        }
    }

    fun setAwake(flag: Boolean) {
        if (flag) {
            if ((m_flags and e_awakeFlag) == 0) {
                m_flags = m_flags or e_awakeFlag
                m_sleepTime = 0f
            }
        } else {
            m_flags = m_flags and e_awakeFlag.inv()
            m_sleepTime = 0f
            m_linearVelocity.setZero()
            m_angularVelocity = 0f
            m_force.setZero()
            m_torque = 0f
        }
    }

    fun isAwake(): Boolean = (m_flags and e_awakeFlag) == e_awakeFlag

    fun setBullet(flag: Boolean) {
        m_flags = if (flag) m_flags or e_bulletFlag else m_flags and e_bulletFlag.inv()
    }

    fun isBullet(): Boolean = (m_flags and e_bulletFlag) == e_bulletFlag

    fun setSleepingAllowed(flag: Boolean) {
        if (flag) {
            m_flags = m_flags or e_autoSleepFlag
        } else {
            m_flags = m_flags and e_autoSleepFlag.inv()
            setAwake(true)
        }
    }

    fun isSleepingAllowed(): Boolean = (m_flags and e_autoSleepFlag) == e_autoSleepFlag

    fun setActive(flag: Boolean) {
        val world = m_world ?: return
        if (world.isLocked()) {
            return
        }
        if (flag == isActive()) {
            return
        }
        if (flag) {
            m_flags = m_flags or e_activeFlag
            val broadPhase = world.m_contactManager.m_broadPhase
            var fixture = m_fixtureList
            while (fixture != null) {
                fixture.createProxies(broadPhase, m_xf)
                fixture = fixture.m_next
            }
            return
        }
        m_flags = m_flags and e_activeFlag.inv()
        val broadPhase = world.m_contactManager.m_broadPhase
        var fixture = m_fixtureList
        while (fixture != null) {
            fixture.destroyProxies(broadPhase)
            fixture = fixture.m_next
        }
        var contactEdge = m_contactList
        while (contactEdge != null) {
            val next = contactEdge.next
            contactEdge.contact?.let(world.m_contactManager::destroy)
            contactEdge = next
        }
        m_contactList = null
    }

    fun isActive(): Boolean = (m_flags and e_activeFlag) == e_activeFlag

    fun setFixedRotation(flag: Boolean) {
        m_flags = if (flag) m_flags or e_fixedRotationFlag else m_flags and e_fixedRotationFlag.inv()
        resetMassData()
    }

    fun isFixedRotation(): Boolean = (m_flags and e_fixedRotationFlag) == e_fixedRotationFlag

    fun setLinearVelocity(v: Vec2) {
        if (m_type == BodyType.STATIC) {
            return
        }
        if (Vec2.dot(v, v) > 0f) {
            setAwake(true)
        }
        m_linearVelocity.set(v)
    }

    fun getLinearVelocity(): Vec2 = m_linearVelocity

    fun setAngularVelocity(w: Float) {
        if (m_type == BodyType.STATIC) {
            return
        }
        if (w * w > 0f) {
            setAwake(true)
        }
        m_angularVelocity = w
    }

    fun getAngularVelocity(): Float = m_angularVelocity

    fun getGravityScale(): Float = m_gravityScale

    fun setGravityScale(gravityScale: Float) {
        m_gravityScale = gravityScale
    }

    fun getWorldPoint(localPoint: Vec2): Vec2 = Vec2().also { getWorldPointToOut(localPoint, it) }

    fun getWorldPointToOut(localPoint: Vec2, out: Vec2) {
        Transform.mulToOut(m_xf, localPoint, out)
    }

    fun getWorldVector(localVector: Vec2): Vec2 = Vec2().also { getWorldVectorToOut(localVector, it) }

    fun getWorldVectorToOut(localVector: Vec2, out: Vec2) {
        Rot.mulToOut(m_xf.q, localVector, out)
    }

    fun getLocalPoint(worldPoint: Vec2): Vec2 = Vec2().also { getLocalPointToOut(worldPoint, it) }

    fun getLocalPointToOut(worldPoint: Vec2, out: Vec2) {
        Transform.mulTransToOut(m_xf, worldPoint, out)
    }

    fun getLocalVector(worldVector: Vec2): Vec2 = Vec2().also { getLocalVectorToOut(worldVector, it) }

    fun getLocalVectorToOut(worldVector: Vec2, out: Vec2) {
        Rot.mulTrans(m_xf.q, worldVector, out)
    }

    fun getLinearVelocityFromWorldPoint(worldPoint: Vec2): Vec2 = Vec2().also {
        getLinearVelocityFromWorldPointToOut(worldPoint, it)
    }

    fun getLinearVelocityFromWorldPointToOut(worldPoint: Vec2, out: Vec2) {
        val tempX = worldPoint.x - m_sweep.c.x
        val tempY = worldPoint.y - m_sweep.c.y
        out.x = -m_angularVelocity * tempY + m_linearVelocity.x
        out.y = m_angularVelocity * tempX + m_linearVelocity.y
    }

    fun getLinearVelocityFromLocalPoint(localPoint: Vec2): Vec2 = Vec2().also {
        getLinearVelocityFromLocalPointToOut(localPoint, it)
    }

    fun getLinearVelocityFromLocalPointToOut(localPoint: Vec2, out: Vec2) {
        getWorldPointToOut(localPoint, out)
        getLinearVelocityFromWorldPointToOut(out, out)
    }

    fun applyForce(force: Vec2, point: Vec2) {
        if (m_type != BodyType.DYNAMIC) {
            return
        }
        if (!isAwake()) {
            setAwake(true)
        }
        m_force.x += force.x
        m_force.y += force.y
        m_torque += (point.x - m_sweep.c.x) * force.y - (point.y - m_sweep.c.y) * force.x
    }

    fun applyForceToCenter(force: Vec2) {
        if (m_type != BodyType.DYNAMIC) {
            return
        }
        if (!isAwake()) {
            setAwake(true)
        }
        m_force.x += force.x
        m_force.y += force.y
    }

    fun applyTorque(torque: Float) {
        if (m_type != BodyType.DYNAMIC) {
            return
        }
        if (!isAwake()) {
            setAwake(true)
        }
        m_torque += torque
    }

    fun applyLinearImpulse(impulse: Vec2, point: Vec2) {
        if (m_type != BodyType.DYNAMIC) {
            return
        }
        if (!isAwake()) {
            setAwake(true)
        }
        m_linearVelocity.x += impulse.x * m_invMass
        m_linearVelocity.y += impulse.y * m_invMass
        m_angularVelocity += m_invI * ((point.x - m_sweep.c.x) * impulse.y - (point.y - m_sweep.c.y) * impulse.x)
    }

    fun applyAngularImpulse(impulse: Float) {
        if (m_type != BodyType.DYNAMIC) {
            return
        }
        if (!isAwake()) {
            setAwake(true)
        }
        m_angularVelocity += m_invI * impulse
    }

    fun shouldCollide(other: Body): Boolean {
        if (m_type != BodyType.DYNAMIC && other.m_type != BodyType.DYNAMIC) {
            return false
        }
        var edge = m_jointList
        while (edge != null) {
            if (edge.other === other && edge.joint?.getCollideConnected() == false) {
                return false
            }
            edge = edge.next
        }
        return this !== other
    }

    fun createFixture(def: FixtureDef): Fixture? {
        val world = m_world
        if (world != null && world.isLocked()) {
            return null
        }
        val fixture = Fixture()
        fixture.create(this, def)
        if ((m_flags and e_activeFlag) == e_activeFlag) {
            world?.m_contactManager?.m_broadPhase?.let { fixture.createProxies(it, m_xf) }
        }
        fixture.m_next = m_fixtureList
        m_fixtureList = fixture
        m_fixtureCount++
        fixture.m_body = this
        if (fixture.m_density > 0f) {
            resetMassData()
        }
        world?.m_flags = (world.m_flags or World.NEW_FIXTURE)
        return fixture
    }

    fun createFixture(shape: Shape, density: Float): Fixture? {
        fixDef.shape = shape
        fixDef.density = density
        return createFixture(fixDef)
    }

    fun destroyFixture(fixture: Fixture) {
        val world = m_world
        if (world != null && world.isLocked()) return
        assert(fixture.m_body === this)
        assert(m_fixtureCount > 0)
        var node = m_fixtureList
        var last: Fixture? = null
        var found = false
        while (node != null) {
            if (node === fixture) {
                found = true
                break
            }
            last = node
            node = node.m_next
        }
        assert(found)
        if (last == null) {
            m_fixtureList = fixture.m_next
        } else {
            last.m_next = fixture.m_next
        }
        var edge = m_contactList
        while (edge != null) {
            val c: Contact? = edge.contact
            edge = edge.next
            if (c == null) continue
            val fixtureA = c.getFixtureA()
            val fixtureB = c.getFixtureB()
            if (fixture === fixtureA || fixture === fixtureB) {
                world?.m_contactManager?.destroy(c)
            }
        }
        if ((m_flags and e_activeFlag) == e_activeFlag) {
            world?.m_contactManager?.m_broadPhase?.let { fixture.destroyProxies(it) }
        }
        fixture.destroy()
        fixture.m_body = null
        fixture.m_next = null
        m_fixtureCount--
        resetMassData()
    }

    fun resetMassData() {
        m_mass = 0f
        m_invMass = 0f
        m_I = 0f
        m_invI = 0f
        m_sweep.localCenter.setZero()

        if (m_type == BodyType.STATIC || m_type == BodyType.KINEMATIC) {
            m_sweep.c0.set(m_xf.p)
            m_sweep.c.set(m_xf.p)
            m_sweep.a0 = m_sweep.a
            return
        }

        localCenter.setZero()
        var fixture = m_fixtureList
        while (fixture != null) {
            if (fixture.m_density == 0f) {
                fixture = fixture.m_next
                continue
            }
            fixture.getMassData(massData)
            m_mass += massData.mass
            temp.set(massData.center).mulLocal(massData.mass)
            localCenter.addLocal(temp)
            m_I += massData.I
            fixture = fixture.m_next
        }

        if (m_mass > 0f) {
            m_invMass = 1f / m_mass
            localCenter.mulLocal(m_invMass)
        } else {
            m_mass = 1f
            m_invMass = 1f
        }

        if (m_I > 0f && (m_flags and e_fixedRotationFlag) == 0) {
            m_I -= m_mass * Vec2.dot(localCenter, localCenter)
            if (m_I > 0f) {
                m_invI = 1f / m_I
            } else {
                m_I = 0f
                m_invI = 0f
            }
        } else {
            m_I = 0f
            m_invI = 0f
        }

        oldCenter.set(m_sweep.c)
        m_sweep.localCenter.set(localCenter)
        Transform.mulToOutUnsafe(m_xf, m_sweep.localCenter, m_sweep.c0)
        m_sweep.c.set(m_sweep.c0)
        temp.set(m_sweep.c).subLocal(oldCenter)
        Vec2.crossToOut(m_angularVelocity, temp, temp)
        m_linearVelocity.addLocal(temp)
    }

    companion object {
        const val e_islandFlag: Int = 0x0001
        const val e_awakeFlag: Int = 0x0002
        const val e_autoSleepFlag: Int = 0x0004
        const val e_bulletFlag: Int = 0x0008
        const val e_fixedRotationFlag: Int = 0x0010
        const val e_activeFlag: Int = 0x0020
        const val e_toiFlag: Int = 0x0040
    }
}
