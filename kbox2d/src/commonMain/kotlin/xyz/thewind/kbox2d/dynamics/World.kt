package xyz.thewind.kbox2d.dynamics

import xyz.thewind.kbox2d.callbacks.ContactFilter
import xyz.thewind.kbox2d.callbacks.ContactListener
import xyz.thewind.kbox2d.callbacks.DebugDraw
import xyz.thewind.kbox2d.callbacks.DestructionListener
import xyz.thewind.kbox2d.callbacks.ParticleDestructionListener
import xyz.thewind.kbox2d.callbacks.QueryCallback
import xyz.thewind.kbox2d.callbacks.RayCastCallback
import xyz.thewind.kbox2d.callbacks.TreeCallback
import xyz.thewind.kbox2d.callbacks.TreeRayCastCallback
import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.RayCastInput
import xyz.thewind.kbox2d.collision.RayCastOutput
import xyz.thewind.kbox2d.collision.TimeOfImpact
import xyz.thewind.kbox2d.collision.broadphase.BroadPhase
import xyz.thewind.kbox2d.collision.broadphase.DefaultBroadPhaseBuffer
import xyz.thewind.kbox2d.collision.broadphase.DynamicTree
import xyz.thewind.kbox2d.collision.shapes.ChainShape
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.collision.shapes.EdgeShape
import xyz.thewind.kbox2d.collision.shapes.PolygonShape
import xyz.thewind.kbox2d.collision.shapes.ShapeType
import xyz.thewind.kbox2d.common.Color3f
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Sweep
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.contacts.Contact
import xyz.thewind.kbox2d.dynamics.contacts.ContactRegister
import xyz.thewind.kbox2d.dynamics.joints.Joint
import xyz.thewind.kbox2d.dynamics.joints.JointDef
import xyz.thewind.kbox2d.particle.ParticleDef
import xyz.thewind.kbox2d.particle.ParticleGroup
import xyz.thewind.kbox2d.particle.ParticleGroupDef
import xyz.thewind.kbox2d.particle.ParticleSystem
import xyz.thewind.kbox2d.pooling.IDynamicStack
import xyz.thewind.kbox2d.pooling.IWorldPool
import xyz.thewind.kbox2d.pooling.normal.DefaultWorldPool

open class World(
    gravity: Vec2 = Vec2(),
    private val pool: IWorldPool = DefaultWorldPool(100, 10),
    broadPhase: BroadPhase = DefaultBroadPhaseBuffer(DynamicTree()),
) {
    var m_flags: Int = 0
    val m_gravity: Vec2 = gravity.clone()
    val m_contactManager: ContactManager = ContactManager(this, broadPhase)
    val m_profile: Profile = Profile()
    private var m_invDt0: Float = 0f
    private var m_warmStarting: Boolean = true
    private var m_continuousPhysics: Boolean = true
    private var m_subStepping: Boolean = false
    private var m_stepComplete: Boolean = true
    private var m_allowSleep: Boolean = true
    private var m_clearForces: Boolean = true
    private val m_particleSystem: ParticleSystem = ParticleSystem(this)
    var m_bodyList: Body? = null
    var m_jointList: Joint? = null
    var m_bodyCount: Int = 0
    var m_jointCount: Int = 0
    var m_destructionListener: DestructionListener? = null
    private var m_particleDestructionListener: ParticleDestructionListener? = null
    private var m_debugDraw: DebugDraw? = null

    private val contactStacks: Array<Array<ContactRegister?>> =
        Array(ShapeType.values().size) { arrayOfNulls(ShapeType.values().size) }

    init {
        initializeRegisters()
    }

    private fun addType(creator: IDynamicStack<Contact>, type1: ShapeType, type2: ShapeType) {
        val register = ContactRegister().apply {
            this.creator = creator
            primary = true
        }
        contactStacks[type1.ordinal][type2.ordinal] = register
        if (type1 != type2) {
            val register2 = ContactRegister().apply {
                this.creator = creator
                primary = false
            }
            contactStacks[type2.ordinal][type1.ordinal] = register2
        }
    }

    private fun initializeRegisters() {
        addType(pool.getCircleContactStack(), ShapeType.CIRCLE, ShapeType.CIRCLE)
        addType(pool.getPolyCircleContactStack(), ShapeType.POLYGON, ShapeType.CIRCLE)
        addType(pool.getPolyContactStack(), ShapeType.POLYGON, ShapeType.POLYGON)
        addType(pool.getEdgeCircleContactStack(), ShapeType.EDGE, ShapeType.CIRCLE)
        addType(pool.getEdgePolyContactStack(), ShapeType.EDGE, ShapeType.POLYGON)
        addType(pool.getChainCircleContactStack(), ShapeType.CHAIN, ShapeType.CIRCLE)
        addType(pool.getChainPolyContactStack(), ShapeType.CHAIN, ShapeType.POLYGON)
    }

    fun createBody(def: BodyDef = BodyDef()): Body? {
        if (isLocked()) {
            return null
        }
        val body = Body(def, this)
        body.m_prev = null
        body.m_next = m_bodyList
        m_bodyList?.m_prev = body
        m_bodyList = body
        m_bodyCount++
        return body
    }

    fun destroyBody(body: Body) {
        if (isLocked()) {
            return
        }
        require(m_bodyCount > 0)

        var jointEdge = body.m_jointList
        while (jointEdge != null) {
            val next = jointEdge.next
            jointEdge.joint?.let(::destroyJoint)
            jointEdge = next
            body.m_jointList = jointEdge
        }
        body.m_jointList = null

        var contactEdge = body.m_contactList
        while (contactEdge != null) {
            val next = contactEdge.next
            contactEdge.contact?.let(m_contactManager::destroy)
            contactEdge = next
        }
        body.m_contactList = null

        var fixture = body.m_fixtureList
        while (fixture != null) {
            val next = fixture.m_next
            m_destructionListener?.sayGoodbye(fixture)
            fixture.destroyProxies(m_contactManager.m_broadPhase)
            fixture.destroy()
            body.m_fixtureList = next
            body.m_fixtureCount--
            fixture = next
        }
        body.m_fixtureList = null
        body.m_fixtureCount = 0

        body.m_prev?.m_next = body.m_next
        body.m_next?.m_prev = body.m_prev
        if (body === m_bodyList) {
            m_bodyList = body.m_next
        }
        body.m_prev = null
        body.m_next = null
        body.m_world = null
        m_bodyCount--
    }

    fun getGravity(): Vec2 = m_gravity

    fun setGravity(gravity: Vec2) {
        m_gravity.set(gravity)
    }

    fun shiftOrigin(newOrigin: Vec2) {
        if (isLocked()) {
            return
        }
        if (newOrigin.x == 0f && newOrigin.y == 0f) {
            return
        }
        var body = m_bodyList
        while (body != null) {
            body.m_xf.p.subLocal(newOrigin)
            body.m_xf0.p.subLocal(newOrigin)
            body.m_sweep.c.subLocal(newOrigin)
            body.m_sweep.c0.subLocal(newOrigin)
            body = body.m_next
        }
        var joint = m_jointList
        while (joint != null) {
            when (joint) {
                is xyz.thewind.kbox2d.dynamics.joints.MouseJoint -> joint.getTarget().subLocal(newOrigin)
                is xyz.thewind.kbox2d.dynamics.joints.PulleyJoint -> {
                    joint.getGroundAnchorA().subLocal(newOrigin)
                    joint.getGroundAnchorB().subLocal(newOrigin)
                }
            }
            joint = joint.m_next
        }
        m_particleSystem.shiftOrigin(newOrigin)
        m_contactManager.m_broadPhase.shiftOrigin(newOrigin)
    }

    fun createParticle(def: ParticleDef): Int = m_particleSystem.createParticle(def)

    fun queryAABB(callback: QueryCallback, aabb: AABB) {
        val broadPhase = m_contactManager.m_broadPhase
        broadPhase.query(TreeCallback { proxyId ->
            val proxy = broadPhase.getUserData(proxyId) as? FixtureProxy ?: return@TreeCallback true
            val fixture = proxy.fixture ?: return@TreeCallback true
            callback.reportFixture(fixture)
        }, aabb)
    }

    fun raycast(callback: RayCastCallback, point1: Vec2, point2: Vec2) {
        val broadPhase = m_contactManager.m_broadPhase
        val input = RayCastInput().apply {
            p1.set(point1)
            p2.set(point2)
            maxFraction = 1f
        }
        broadPhase.raycast(TreeRayCastCallback { subInput, nodeId ->
            val proxy = broadPhase.getUserData(nodeId) as? FixtureProxy ?: return@TreeRayCastCallback subInput.maxFraction
            val fixture = proxy.fixture ?: return@TreeRayCastCallback subInput.maxFraction
            val output = RayCastOutput()
            if (!fixture.raycast(output, subInput, proxy.childIndex)) {
                return@TreeRayCastCallback subInput.maxFraction
            }
            val point = Vec2(
                (1f - output.fraction) * subInput.p1.x + output.fraction * subInput.p2.x,
                (1f - output.fraction) * subInput.p1.y + output.fraction * subInput.p2.y,
            )
            callback.reportFixture(fixture, point, output.normal, output.fraction)
        }, input)
    }

    fun destroyParticle(index: Int, callDestructionListener: Boolean = false) {
        m_particleSystem.destroyParticle(index, callDestructionListener)
    }

    fun createParticleGroup(def: ParticleGroupDef): ParticleGroup = m_particleSystem.createParticleGroup(def)

    fun destroyParticlesInGroup(group: ParticleGroup, callDestructionListener: Boolean = false) {
        m_particleSystem.destroyParticlesInGroup(group, callDestructionListener)
    }

    fun destroyParticlesInShape(
        shape: xyz.thewind.kbox2d.collision.shapes.Shape,
        xf: xyz.thewind.kbox2d.common.Transform,
        callDestructionListener: Boolean = false,
    ): Int = m_particleSystem.destroyParticlesInShape(shape, xf, callDestructionListener)

    fun computeDepthForGroup(group: ParticleGroup) {
        m_particleSystem.computeDepthForGroup(group)
    }

    fun joinParticleGroups(groupA: ParticleGroup, groupB: ParticleGroup) {
        m_particleSystem.joinParticleGroups(groupA, groupB)
    }

    fun queryParticleAABB(callback: xyz.thewind.kbox2d.callbacks.ParticleQueryCallback, aabb: xyz.thewind.kbox2d.collision.AABB) {
        m_particleSystem.queryAABB(callback, aabb)
    }

    fun raycastParticle(
        callback: xyz.thewind.kbox2d.callbacks.ParticleRaycastCallback,
        point1: Vec2,
        point2: Vec2,
    ) {
        m_particleSystem.raycast(callback, point1, point2)
    }

    fun getParticleCount(): Int = m_particleSystem.getParticleCount()

    fun getParticlePositionBuffer(): Array<Vec2> = m_particleSystem.getParticlePositionBuffer()

    fun getParticleVelocityBuffer(): Array<Vec2> = m_particleSystem.getParticleVelocityBuffer()

    fun getParticleColorBuffer(): Array<xyz.thewind.kbox2d.particle.ParticleColor?> = m_particleSystem.getParticleColorBuffer()

    fun getParticleGroupBuffer(): Array<ParticleGroup?> = m_particleSystem.getParticleGroupBuffer()

    fun getParticleUserDataBuffer(): Array<Any?> = m_particleSystem.getParticleUserDataBuffer()

    fun getParticleDepthBuffer(): FloatArray = m_particleSystem.getParticleDepthBuffer()

    fun getParticleContactBuffer(): Array<xyz.thewind.kbox2d.particle.ParticleContact> = m_particleSystem.getParticleContactBuffer()

    fun getParticleBodyContactBuffer(): Array<xyz.thewind.kbox2d.particle.ParticleBodyContact> = m_particleSystem.getParticleBodyContactBuffer()

    fun getParticleContactCount(): Int = m_particleSystem.getParticleContactCount()

    fun getParticleBodyContactCount(): Int = m_particleSystem.getParticleBodyContactCount()

    fun getParticleFlagsBuffer(): IntArray = m_particleSystem.getParticleFlagsBuffer()

    fun getParticleGroupList(): ParticleGroup? = m_particleSystem.getParticleGroupList()

    fun getParticleGroupCount(): Int = m_particleSystem.getParticleGroupCount()

    fun computeParticleCollisionEnergy(): Float = m_particleSystem.computeParticleCollisionEnergy()

    fun setParticleMaxCount(count: Int) { m_particleSystem.setParticleMaxCount(count) }

    fun getParticleMaxCount(): Int = m_particleSystem.getParticleMaxCount()

    fun setParticleDensity(density: Float) { m_particleSystem.setParticleDensity(density) }

    fun getParticleDensity(): Float = m_particleSystem.getParticleDensity()

    fun setParticleGravityScale(scale: Float) { m_particleSystem.setParticleGravityScale(scale) }

    fun getParticleGravityScale(): Float = m_particleSystem.getParticleGravityScale()

    fun setParticleDamping(damping: Float) { m_particleSystem.setParticleDamping(damping) }

    fun getParticleDamping(): Float = m_particleSystem.getParticleDamping()

    fun setParticleRadius(radius: Float) { m_particleSystem.setParticleRadius(radius) }

    fun getParticleRadius(): Float = m_particleSystem.getParticleRadius()

    fun createJoint(def: JointDef): Joint? {
        if (isLocked()) {
            return null
        }
        val joint = Joint.create(this, def)
        joint.m_prev = null
        joint.m_next = m_jointList
        m_jointList?.m_prev = joint
        m_jointList = joint
        m_jointCount++

        joint.m_edgeA.joint = joint
        joint.m_edgeA.other = joint.getBodyB()
        joint.m_edgeA.prev = null
        joint.m_edgeA.next = joint.getBodyA().m_jointList
        joint.getBodyA().m_jointList?.prev = joint.m_edgeA
        joint.getBodyA().m_jointList = joint.m_edgeA

        joint.m_edgeB.joint = joint
        joint.m_edgeB.other = joint.getBodyA()
        joint.m_edgeB.prev = null
        joint.m_edgeB.next = joint.getBodyB().m_jointList
        joint.getBodyB().m_jointList?.prev = joint.m_edgeB
        joint.getBodyB().m_jointList = joint.m_edgeB

        val bodyA = requireNotNull(def.bodyA)
        val bodyB = requireNotNull(def.bodyB)
        if (!def.collideConnected) {
            var edge = bodyB.getContactList()
            while (edge != null) {
                if (edge.other === bodyA) {
                    edge.contact?.flagForFiltering()
                }
                edge = edge.next
            }
        }
        return joint
    }

    fun destroyJoint(joint: Joint) {
        if (isLocked()) {
            return
        }
        val collideConnected = joint.getCollideConnected()
        joint.m_prev?.m_next = joint.m_next
        joint.m_next?.m_prev = joint.m_prev
        if (joint === m_jointList) {
            m_jointList = joint.m_next
        }
        val bodyA = joint.getBodyA()
        val bodyB = joint.getBodyB()
        bodyA.setAwake(true)
        bodyB.setAwake(true)
        joint.m_edgeA.prev?.next = joint.m_edgeA.next
        joint.m_edgeA.next?.prev = joint.m_edgeA.prev
        if (joint.m_edgeA === bodyA.m_jointList) {
            bodyA.m_jointList = joint.m_edgeA.next
        }
        joint.m_edgeA.prev = null
        joint.m_edgeA.next = null
        joint.m_edgeB.prev?.next = joint.m_edgeB.next
        joint.m_edgeB.next?.prev = joint.m_edgeB.prev
        if (joint.m_edgeB === bodyB.m_jointList) {
            bodyB.m_jointList = joint.m_edgeB.next
        }
        joint.m_edgeB.prev = null
        joint.m_edgeB.next = null
        Joint.destroy(joint)
        m_jointCount--
        if (!collideConnected) {
            var edge = bodyB.getContactList()
            while (edge != null) {
                if (edge.other === bodyA) {
                    edge.contact?.flagForFiltering()
                }
                edge = edge.next
            }
        }
    }

    fun step(dt: Float, velocityIterations: Int, positionIterations: Int) {
        if ((m_flags and NEW_FIXTURE) == NEW_FIXTURE) {
            m_contactManager.findNewContacts()
            m_flags = m_flags and NEW_FIXTURE.inv()
        }

        m_flags = m_flags or LOCKED
        val step = TimeStep().apply {
            this.dt = dt
            inv_dt = if (dt > 0f) 1f / dt else 0f
            dtRatio = m_invDt0 * dt
            this.velocityIterations = velocityIterations
            this.positionIterations = positionIterations
            warmStarting = m_warmStarting
        }

        m_contactManager.collide()
        if (step.dt > 0f) {
            m_particleSystem.solve(step)
            solve(step)
            if (m_continuousPhysics) {
                solveTOI(step)
            }
        }
        if (m_clearForces) {
            clearForces()
        }
        m_invDt0 = step.inv_dt
        m_flags = m_flags and LOCKED.inv()
    }

    fun setAllowSleep(flag: Boolean) {
        m_allowSleep = flag
        if (!flag) {
            var body = m_bodyList
            while (body != null) {
                body.setAwake(true)
                body = body.m_next
            }
        }
    }

    fun setWarmStarting(flag: Boolean) {
        m_warmStarting = flag
    }

    fun setContinuousPhysics(flag: Boolean) {
        m_continuousPhysics = flag
    }

    fun setSubStepping(flag: Boolean) {
        m_subStepping = flag
    }

    private fun solveTOI(step: TimeStep) {
        if (m_contactManager.m_contactCount == 0) {
            return
        }

        val island = Island(m_contactManager.m_contactListener)
        island.init(2 * Settings.maxTOIContacts, Settings.maxTOIContacts, 0)
        val toiInput = TimeOfImpact.TOIInput()
        val toiOutput = TimeOfImpact.TOIOutput()
        val subStep = TimeStep()
        val tempBodies = arrayOfNulls<Body>(2)
        val backup1 = Sweep()
        val backup2 = Sweep()

        if (m_stepComplete) {
            var body = m_bodyList
            while (body != null) {
                body.m_flags = body.m_flags and Body.e_islandFlag.inv()
                body.m_sweep.alpha0 = 0f
                body = body.m_next
            }
            var contact = m_contactManager.m_contactList
            while (contact != null) {
                contact.m_flags = contact.m_flags and (Contact.TOI_FLAG or Contact.ISLAND_FLAG).inv()
                contact.m_toiCount = 0f
                contact.m_toi = 1f
                contact = contact.m_next
            }
        }

        while (true) {
            var minContact: Contact? = null
            var minAlpha = 1f
            var contact = m_contactManager.m_contactList
            while (contact != null) {
                if (!contact.isEnabled()) {
                    contact = contact.m_next
                    continue
                }
                if (contact.m_toiCount > Settings.maxSubSteps) {
                    contact = contact.m_next
                    continue
                }
                val alpha = if ((contact.m_flags and Contact.TOI_FLAG) == Contact.TOI_FLAG) {
                    contact.m_toi
                } else {
                    val fixtureA = contact.getFixtureA()
                    val fixtureB = contact.getFixtureB()
                    if (fixtureA.isSensor() || fixtureB.isSensor()) {
                        contact = contact.m_next
                        continue
                    }
                    val bodyA = fixtureA.getBody()!!
                    val bodyB = fixtureB.getBody()!!
                    val typeA = bodyA.m_type
                    val typeB = bodyB.m_type
                    val activeA = bodyA.isAwake() && typeA != BodyType.STATIC
                    val activeB = bodyB.isAwake() && typeB != BodyType.STATIC
                    if (!activeA && !activeB) {
                        contact = contact.m_next
                        continue
                    }
                    val collideA = bodyA.isBullet() || typeA != BodyType.DYNAMIC
                    val collideB = bodyB.isBullet() || typeB != BodyType.DYNAMIC
                    if (!collideA && !collideB) {
                        contact = contact.m_next
                        continue
                    }
                    var alpha0 = bodyA.m_sweep.alpha0
                    if (bodyA.m_sweep.alpha0 < bodyB.m_sweep.alpha0) {
                        alpha0 = bodyB.m_sweep.alpha0
                        bodyA.m_sweep.advance(alpha0)
                    } else if (bodyB.m_sweep.alpha0 < bodyA.m_sweep.alpha0) {
                        alpha0 = bodyA.m_sweep.alpha0
                        bodyB.m_sweep.advance(alpha0)
                    }
                    toiInput.proxyA.set(fixtureA.getShape(), contact.getChildIndexA())
                    toiInput.proxyB.set(fixtureB.getShape(), contact.getChildIndexB())
                    toiInput.sweepA.set(bodyA.m_sweep)
                    toiInput.sweepB.set(bodyB.m_sweep)
                    toiInput.tMax = 1f
                    pool.getTimeOfImpact().timeOfImpact(toiOutput, toiInput)
                    val resolvedAlpha = if (toiOutput.state == TimeOfImpact.TOIOutputState.TOUCHING) {
                        kotlin.math.min(alpha0 + (1f - alpha0) * toiOutput.t, 1f)
                    } else {
                        1f
                    }
                    contact.m_toi = resolvedAlpha
                    contact.m_flags = contact.m_flags or Contact.TOI_FLAG
                    resolvedAlpha
                }
                if (alpha < minAlpha) {
                    minContact = contact
                    minAlpha = alpha
                }
                contact = contact.m_next
            }

            if (minContact == null || 1f - 10f * Settings.EPSILON < minAlpha) {
                m_stepComplete = true
                break
            }

            val fixtureA = minContact.getFixtureA()
            val fixtureB = minContact.getFixtureB()
            val bodyA = fixtureA.getBody()!!
            val bodyB = fixtureB.getBody()!!
            backup1.set(bodyA.m_sweep)
            backup2.set(bodyB.m_sweep)
            bodyA.advance(minAlpha)
            bodyB.advance(minAlpha)
            minContact.update(m_contactManager.m_contactListener)
            minContact.m_flags = minContact.m_flags and Contact.TOI_FLAG.inv()
            minContact.m_toiCount += 1f

            if (!minContact.isEnabled() || !minContact.isTouching()) {
                minContact.setEnabled(false)
                bodyA.m_sweep.set(backup1)
                bodyB.m_sweep.set(backup2)
                bodyA.synchronizeTransform()
                bodyB.synchronizeTransform()
                continue
            }

            bodyA.setAwake(true)
            bodyB.setAwake(true)
            island.init(2 * Settings.maxTOIContacts, Settings.maxTOIContacts, 0)
            island.add(bodyA)
            island.add(bodyB)
            island.add(minContact)
            bodyA.m_flags = bodyA.m_flags or Body.e_islandFlag
            bodyB.m_flags = bodyB.m_flags or Body.e_islandFlag
            minContact.m_flags = minContact.m_flags or Contact.ISLAND_FLAG

            tempBodies[0] = bodyA
            tempBodies[1] = bodyB
            for (i in 0..1) {
                val body = tempBodies[i] ?: continue
                if (body.m_type != BodyType.DYNAMIC) continue
                var ce = body.m_contactList
                while (ce != null) {
                    if (island.m_bodyCount == island.m_bodyCapacity || island.m_contactCount == island.m_contactCapacity) {
                        break
                    }
                    val contact2 = ce.contact!!
                    if ((contact2.m_flags and Contact.ISLAND_FLAG) != 0) {
                        ce = ce.next
                        continue
                    }
                    val other = ce.other!!
                    if (other.m_type == BodyType.DYNAMIC && !body.isBullet() && !other.isBullet()) {
                        ce = ce.next
                        continue
                    }
                    if (contact2.getFixtureA().isSensor() || contact2.getFixtureB().isSensor()) {
                        ce = ce.next
                        continue
                    }
                    backup1.set(other.m_sweep)
                    if ((other.m_flags and Body.e_islandFlag) == 0) {
                        other.advance(minAlpha)
                    }
                    contact2.update(m_contactManager.m_contactListener)
                    if (!contact2.isEnabled()) {
                        other.m_sweep.set(backup1)
                        other.synchronizeTransform()
                        ce = ce.next
                        continue
                    }
                    if (!contact2.isTouching()) {
                        other.m_sweep.set(backup1)
                        other.synchronizeTransform()
                        ce = ce.next
                        continue
                    }
                    contact2.m_flags = contact2.m_flags or Contact.ISLAND_FLAG
                    island.add(contact2)
                    if ((other.m_flags and Body.e_islandFlag) == 0) {
                        other.m_flags = other.m_flags or Body.e_islandFlag
                        if (other.m_type != BodyType.STATIC) {
                            other.setAwake(true)
                        }
                        island.add(other)
                    }
                    ce = ce.next
                }
            }

            subStep.dt = (1f - minAlpha) * step.dt
            subStep.inv_dt = if (subStep.dt > 0f) 1f / subStep.dt else 0f
            subStep.dtRatio = 1f
            subStep.positionIterations = 20
            subStep.velocityIterations = step.velocityIterations
            subStep.warmStarting = false
            island.solveTOI(subStep, bodyA.m_islandIndex, bodyB.m_islandIndex)

            for (i in 0 until island.m_bodyCount) {
                val body = island.m_bodies[i]!!
                body.m_flags = body.m_flags and Body.e_islandFlag.inv()
                if (body.m_type != BodyType.DYNAMIC) {
                    continue
                }
                body.synchronizeFixtures()
                var ce = body.m_contactList
                while (ce != null) {
                    ce.contact?.let { c ->
                        c.m_flags = c.m_flags and (Contact.TOI_FLAG or Contact.ISLAND_FLAG).inv()
                    }
                    ce = ce.next
                }
            }

            m_contactManager.findNewContacts()
            if (m_subStepping) {
                m_stepComplete = false
                break
            }
        }
    }

    private fun solve(step: TimeStep) {
        if (m_bodyCount == 0) {
            return
        }

        var body = m_bodyList
        while (body != null) {
            body.m_flags = body.m_flags and Body.e_islandFlag.inv()
            body = body.m_next
        }
        var contact = m_contactManager.m_contactList
        while (contact != null) {
            contact.m_flags = contact.m_flags and Contact.ISLAND_FLAG.inv()
            contact = contact.m_next
        }

        val island = Island(m_contactManager.m_contactListener)
        island.init(m_bodyCount, m_contactManager.m_contactCount, m_jointCount)

        body = m_bodyList
        while (body != null) {
            if (body.isActive() && (body.isAwake() || body.m_type == BodyType.STATIC)) {
                island.add(body)
                body.m_flags = body.m_flags or Body.e_islandFlag
            }
            body = body.m_next
        }

        var joint = m_jointList
        while (joint != null) {
            joint.m_islandFlag = false
            joint = joint.m_next
        }

        joint = m_jointList
        while (joint != null) {
            if (joint.isActive()) {
                island.add(joint)
                joint.m_islandFlag = true
            }
            joint = joint.m_next
        }

        contact = m_contactManager.m_contactList
        while (contact != null) {
            if (contact.isEnabled() && contact.isTouching()) {
                island.add(contact)
                contact.m_flags = contact.m_flags or Contact.ISLAND_FLAG
            }
            contact = contact.m_next
        }

        if (island.m_bodyCount > 0) {
            island.solve(m_profile, step, m_gravity, m_allowSleep)
        }

        body = m_bodyList
        while (body != null) {
            if (body.m_type != BodyType.STATIC) {
                body.synchronizeFixtures()
            }
            body = body.m_next
        }
        m_contactManager.findNewContacts()
    }

    fun popContact(fixtureA: Fixture, indexA: Int, fixtureB: Fixture, indexB: Int): Contact? {
        val type1 = fixtureA.getType()
        val type2 = fixtureB.getType()
        val reg = contactStacks[type1.ordinal][type2.ordinal] ?: return null
        val contact = reg.creator!!.pop()
        if (reg.primary) {
            contact.init(fixtureA, indexA, fixtureB, indexB)
        } else {
            contact.init(fixtureB, indexB, fixtureA, indexA)
        }
        return contact
    }

    fun pushContact(contact: Contact) {
        val fixtureA = contact.getFixtureA()
        val fixtureB = contact.getFixtureB()
        if (contact.m_manifold.pointCount > 0 && !fixtureA.isSensor() && !fixtureB.isSensor()) {
            fixtureA.getBody()?.setAwake(true)
            fixtureB.getBody()?.setAwake(true)
        }
        val creator = contactStacks[fixtureA.getType().ordinal][fixtureB.getType().ordinal]!!.creator!!
        creator.push(contact)
    }

    fun dump(): String = buildString {
        appendLine("World dump:")
        appendLine(" gravity=$m_gravity")
        appendLine(" bodies=$m_bodyCount joints=$m_jointCount contacts=${m_contactManager.m_contactCount} proxies=${m_contactManager.m_broadPhase.getProxyCount()}")
        appendLine(" flags: allowSleep=$m_allowSleep warmStarting=$m_warmStarting continuousPhysics=$m_continuousPhysics subStepping=$m_subStepping autoClearForces=$m_clearForces")
        val profileLines = mutableListOf<String>()
        m_profile.toDebugStrings(profileLines)
        profileLines.forEach(::appendLine)
    }

    fun getProfile(): Profile = m_profile

    fun clearForces() {
        var body = m_bodyList
        while (body != null) {
            body.m_force.setZero()
            body.m_torque = 0f
            body = body.m_next
        }
    }

    fun setAutoClearForces(flag: Boolean) {
        m_clearForces = flag
    }

    fun getAutoClearForces(): Boolean = m_clearForces

    fun setDebugDraw(debugDraw: DebugDraw?) {
        m_debugDraw = debugDraw
    }

    fun getDebugDraw(): DebugDraw? = m_debugDraw

    fun drawDebugData() {
        val debugDraw = m_debugDraw ?: return
        val flags = debugDraw.getFlags()
        if ((flags and DebugDraw.e_shapeBit) != 0) {
            var body = m_bodyList
            while (body != null) {
                var fixture = body.getFixtureList()
                val color = getBodyDebugColor(body)
                while (fixture != null) {
                    drawShape(debugDraw, fixture, body, color, flags)
                    fixture = fixture.getNext()
                }
                body = body.getNext()
            }
        }
        if ((flags and DebugDraw.e_jointBit) != 0) {
            var joint = m_jointList
            while (joint != null) {
                drawJoint(debugDraw, joint)
                joint = joint.getNext()
            }
        }
        if ((flags and DebugDraw.e_pairBit) != 0) {
            var contact = m_contactManager.m_contactList
            while (contact != null) {
                val centerA = contact.getFixtureA().getAABB(contact.getChildIndexA()).getCenter()
                val centerB = contact.getFixtureB().getAABB(contact.getChildIndexB()).getCenter()
                debugDraw.drawSegment(centerA, centerB, Color3f(0.3f, 0.9f, 0.9f))
                contact = contact.getNext()
            }
        }
        if ((flags and DebugDraw.e_aabbBit) != 0) {
            var body = m_bodyList
            while (body != null) {
                var fixture = body.getFixtureList()
                while (fixture != null) {
                    drawFixtureAabbs(debugDraw, fixture)
                    fixture = fixture.getNext()
                }
                body = body.getNext()
            }
        }
        if ((flags and DebugDraw.e_centerOfMassBit) != 0) {
            var body = m_bodyList
            while (body != null) {
                val transform = Transform(body.getTransform())
                transform.p.set(body.getWorldCenter())
                debugDraw.drawTransform(transform)
                body = body.getNext()
            }
        }
        if ((flags and DebugDraw.e_dynamicTreeBit) != 0) {
            m_contactManager.m_broadPhase.drawTree(debugDraw)
        }
        debugDraw.flush()
    }

    private fun getBodyDebugColor(body: Body): Color3f = when {
        !body.isActive() -> Color3f(0.5f, 0.5f, 0.3f)
        body.getType() == BodyType.STATIC -> Color3f(0.5f, 0.9f, 0.5f)
        body.getType() == BodyType.KINEMATIC -> Color3f(0.5f, 0.5f, 0.9f)
        !body.isAwake() -> Color3f(0.6f, 0.6f, 0.6f)
        else -> Color3f(0.9f, 0.7f, 0.7f)
    }

    private fun drawShape(debugDraw: DebugDraw, fixture: Fixture, body: Body, color: Color3f, flags: Int) {
        val wireframe = (flags and DebugDraw.e_wireframeDrawingBit) != 0
        when (val shape = fixture.getShape()) {
            is CircleShape -> {
                val center = body.getWorldPoint(shape.m_p)
                val axis = body.getWorldVector(Vec2(1f, 0f))
                if (wireframe) {
                    debugDraw.drawCircle(center, shape.m_radius, color)
                } else {
                    debugDraw.drawSolidCircle(center, shape.m_radius, axis, color)
                }
            }
            is PolygonShape -> {
                val vertices = Array(shape.m_count) { Vec2() }
                for (index in 0 until shape.m_count) {
                    body.getWorldPointToOut(shape.m_vertices[index], vertices[index])
                }
                if (wireframe) {
                    debugDraw.drawPolygon(vertices, shape.m_count, color)
                } else {
                    debugDraw.drawSolidPolygon(vertices, shape.m_count, color)
                }
            }
            is EdgeShape -> {
                debugDraw.drawSegment(body.getWorldPoint(shape.m_vertex1), body.getWorldPoint(shape.m_vertex2), color)
            }
            is ChainShape -> {
                val vertices = shape.m_vertices ?: return
                for (index in 0 until shape.m_count - 1) {
                    debugDraw.drawSegment(body.getWorldPoint(vertices[index]), body.getWorldPoint(vertices[index + 1]), color)
                }
            }
            else -> Unit
        }
    }

    private fun drawJoint(debugDraw: DebugDraw, joint: Joint) {
        val anchorA = Vec2()
        val anchorB = Vec2()
        joint.getAnchorA(anchorA)
        joint.getAnchorB(anchorB)
        val centerA = joint.getBodyA().getWorldCenter()
        val centerB = joint.getBodyB().getWorldCenter()
        val color = Color3f(0.5f, 0.8f, 0.8f)
        debugDraw.drawSegment(centerA, anchorA, color)
        debugDraw.drawSegment(anchorA, anchorB, color)
        debugDraw.drawSegment(centerB, anchorB, color)
    }

    private fun drawFixtureAabbs(debugDraw: DebugDraw, fixture: Fixture) {
        val color = Color3f(0.9f, 0.3f, 0.9f)
        for (index in 0 until fixture.m_proxyCount) {
            val vertices = Array(4) { Vec2() }
            fixture.getAABB(index).getVertices(vertices)
            debugDraw.drawPolygon(vertices, 4, color)
        }
    }

    fun getPool(): IWorldPool = pool

    fun getBodyList(): Body? = m_bodyList

    fun getJointList(): Joint? = m_jointList

    fun getContactList(): Contact? = m_contactManager.m_contactList

    fun getBodyCount(): Int = m_bodyCount

    fun getJointCount(): Int = m_jointCount

    fun getContactCount(): Int = m_contactManager.m_contactCount

    fun getProxyCount(): Int = m_contactManager.m_broadPhase.getProxyCount()

    fun getTreeHeight(): Int = m_contactManager.m_broadPhase.getTreeHeight()

    fun getTreeBalance(): Int = m_contactManager.m_broadPhase.getTreeBalance()

    fun getTreeQuality(): Float = m_contactManager.m_broadPhase.getTreeQuality()

    fun isLocked(): Boolean = (m_flags and LOCKED) == LOCKED

    fun setContactFilter(filter: ContactFilter) {
        m_contactManager.m_contactFilter = filter
    }

    fun setContactListener(listener: ContactListener) {
        m_contactManager.m_contactListener = listener
    }

    fun setDestructionListener(listener: DestructionListener?) {
        m_destructionListener = listener
    }

    fun getParticleDestructionListener(): ParticleDestructionListener? = m_particleDestructionListener

    fun setParticleDestructionListener(listener: ParticleDestructionListener?) {
        m_particleDestructionListener = listener
    }

    companion object {
        const val NEW_FIXTURE: Int = 0x0001
        const val LOCKED: Int = 0x0002
    }
}
