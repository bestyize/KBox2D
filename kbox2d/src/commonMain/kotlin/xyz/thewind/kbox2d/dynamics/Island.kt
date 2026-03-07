package xyz.thewind.kbox2d.dynamics

import xyz.thewind.kbox2d.callbacks.ContactImpulse
import xyz.thewind.kbox2d.callbacks.ContactListener
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.contacts.Contact
import xyz.thewind.kbox2d.dynamics.joints.Joint
import xyz.thewind.kbox2d.dynamics.contacts.ContactSolver
import xyz.thewind.kbox2d.dynamics.contacts.Position
import xyz.thewind.kbox2d.dynamics.contacts.Velocity

class Island(
    private val listener: ContactListener?,
) {
    private val contactSolver = ContactSolver()
    private val solverData = SolverData()
    private val impulse = ContactImpulse()

    var m_bodyCapacity: Int = 0
    var m_contactCapacity: Int = 0
    var m_bodyCount: Int = 0
    var m_contactCount: Int = 0
    var m_jointCount: Int = 0
    var m_bodies: Array<Body?> = emptyArray()
    var m_contacts: Array<Contact?> = emptyArray()
    var m_joints: Array<Joint?> = emptyArray()
    var m_positions: Array<Position> = emptyArray()
    var m_velocities: Array<Velocity> = emptyArray()

    fun init(bodyCapacity: Int, contactCapacity: Int, jointCapacity: Int) {
        m_bodyCapacity = bodyCapacity
        m_contactCapacity = contactCapacity
        if (m_bodies.size < bodyCapacity) {
            m_bodies = arrayOfNulls(bodyCapacity)
            m_positions = Array(bodyCapacity) { Position() }
            m_velocities = Array(bodyCapacity) { Velocity() }
        }
        if (m_contacts.size < contactCapacity) {
            m_contacts = arrayOfNulls(contactCapacity)
        }
        if (m_joints.size < jointCapacity) {
            m_joints = arrayOfNulls(jointCapacity)
        }
        m_bodyCount = 0
        m_contactCount = 0
        m_jointCount = 0
    }

    fun add(body: Body) {
        body.m_islandIndex = m_bodyCount
        m_bodies[m_bodyCount++] = body
    }

    fun add(contact: Contact) {
        m_contacts[m_contactCount++] = contact
    }

    fun add(joint: Joint) {
        m_joints[m_jointCount++] = joint
    }

    fun solve(profile: Profile, step: TimeStep, gravity: Vec2, allowSleep: Boolean) {
        for (index in 0 until m_bodyCount) {
            val body = m_bodies[index]!!
            val position = m_positions[index]
            val velocity = m_velocities[index]
            val c = body.m_sweep.c
            var a = body.m_sweep.a
            val v = body.m_linearVelocity
            var w = body.m_angularVelocity

            body.m_sweep.c0.set(body.m_sweep.c)
            body.m_sweep.a0 = body.m_sweep.a
            body.m_xf0.set(body.m_xf)

            if (body.m_type == BodyType.DYNAMIC) {
                v.x += step.dt * (body.m_gravityScale * gravity.x + body.m_invMass * body.m_force.x)
                v.y += step.dt * (body.m_gravityScale * gravity.y + body.m_invMass * body.m_force.y)
                w += step.dt * body.m_invI * body.m_torque
                v.mulLocal(1f / (1f + step.dt * body.m_linearDamping))
                w *= 1f / (1f + step.dt * body.m_angularDamping)
            }

            position.c.set(c)
            position.a = a
            velocity.v.set(v)
            velocity.w = w
        }

        solverData.step = step
        solverData.positions = m_positions
        solverData.velocities = m_velocities

        val solverDef = ContactSolver.ContactSolverDef().apply {
            this.step = step
            contacts = m_contacts
            count = m_contactCount
            positions = m_positions
            velocities = m_velocities
        }
        contactSolver.init(solverDef)
        if (step.warmStarting) {
            contactSolver.warmStart()
        }

        for (index in 0 until m_jointCount) {
            m_joints[index]!!.initVelocityConstraints(solverData)
        }

        profile.solveVelocity.startAccum()
        repeat(step.velocityIterations) {
            for (index in 0 until m_jointCount) {
                m_joints[index]!!.solveVelocityConstraints(solverData)
            }
            contactSolver.solveVelocityConstraints()
        }
        contactSolver.storeImpulses()
        profile.solveVelocity.endAccum()

        profile.solvePosition.startAccum()
        for (index in 0 until m_bodyCount) {
            val body = m_bodies[index]!!
            if (body.m_type == BodyType.STATIC) {
                m_positions[index].c.set(body.m_sweep.c)
                m_positions[index].a = body.m_sweep.a
                m_velocities[index].v.setZero()
                m_velocities[index].w = 0f
                continue
            }

            val c = m_positions[index].c
            var a = m_positions[index].a
            val v = m_velocities[index].v
            var w = m_velocities[index].w

            val translationX = step.dt * v.x
            val translationY = step.dt * v.y
            if (translationX * translationX + translationY * translationY > Settings.maxTranslationSquared) {
                val ratio = Settings.maxTranslation / MathUtils.sqrt(translationX * translationX + translationY * translationY)
                v.mulLocal(ratio)
            }
            val rotation = step.dt * w
            if (rotation * rotation > Settings.maxRotationSquared) {
                w *= Settings.maxRotation / MathUtils.abs(rotation)
            }

            c.x += step.dt * v.x
            c.y += step.dt * v.y
            a += step.dt * w
            m_positions[index].a = a
            m_velocities[index].w = w
        }

        for (iteration in 0 until step.positionIterations) {
            val contactsOkay = contactSolver.solvePositionConstraints()
            var jointsOkay = true
            for (index in 0 until m_jointCount) {
                jointsOkay = m_joints[index]!!.solvePositionConstraints(solverData) && jointsOkay
            }
            if (contactsOkay && jointsOkay) {
                break
            }
        }
        profile.solvePosition.endAccum()

        for (index in 0 until m_bodyCount) {
            val body = m_bodies[index]!!
            if (body.m_type == BodyType.STATIC) {
                m_positions[index].c.set(body.m_sweep.c)
                m_positions[index].a = body.m_sweep.a
                m_velocities[index].v.setZero()
                m_velocities[index].w = 0f
            }
        }

        for (index in 0 until m_bodyCount) {
            val body = m_bodies[index]!!
            body.m_sweep.c.set(m_positions[index].c)
            body.m_sweep.a = m_positions[index].a
            if (body.m_type == BodyType.STATIC) {
                body.m_linearVelocity.setZero()
                body.m_angularVelocity = 0f
            } else {
                body.m_linearVelocity.set(m_velocities[index].v)
                body.m_angularVelocity = m_velocities[index].w
            }
            body.synchronizeTransform()
        }

        report()
        if (allowSleep) {
            solveSleep(step.dt)
        }
    }


    fun solveTOI(subStep: TimeStep, toiIndexA: Int, toiIndexB: Int) {
        require(toiIndexA < m_bodyCount)
        require(toiIndexB < m_bodyCount)

        for (index in 0 until m_bodyCount) {
            val body = m_bodies[index]!!
            m_positions[index].c.set(body.m_sweep.c)
            m_positions[index].a = body.m_sweep.a
            m_velocities[index].v.set(body.m_linearVelocity)
            m_velocities[index].w = body.m_angularVelocity
        }

        val solverDef = ContactSolver.ContactSolverDef().apply {
            step = subStep
            contacts = m_contacts
            count = m_contactCount
            positions = m_positions
            velocities = m_velocities
        }
        contactSolver.init(solverDef)

        repeat(subStep.positionIterations) {
            if (contactSolver.solveTOIPositionConstraints(toiIndexA, toiIndexB)) {
                return@repeat
            }
        }

        m_bodies[toiIndexA]!!.m_sweep.c0.set(m_positions[toiIndexA].c)
        m_bodies[toiIndexA]!!.m_sweep.a0 = m_positions[toiIndexA].a
        m_bodies[toiIndexB]!!.m_sweep.c0.set(m_positions[toiIndexB].c)
        m_bodies[toiIndexB]!!.m_sweep.a0 = m_positions[toiIndexB].a

        for (index in 0 until m_bodyCount) {
            val body = m_bodies[index]!!
            body.m_sweep.c.set(m_positions[index].c)
            body.m_sweep.a = m_positions[index].a
            body.synchronizeTransform()
        }

        contactSolver.init(solverDef)
        repeat(subStep.velocityIterations) {
            contactSolver.solveVelocityConstraints()
        }

        for (index in 0 until m_bodyCount) {
            val body = m_bodies[index]!!
            if (body.m_type == BodyType.STATIC) {
                m_positions[index].c.set(body.m_sweep.c)
                m_positions[index].a = body.m_sweep.a
                m_velocities[index].v.setZero()
                m_velocities[index].w = 0f
                continue
            }
            val c = m_positions[index].c
            var a = m_positions[index].a
            val v = m_velocities[index].v
            var w = m_velocities[index].w
            val translationX = subStep.dt * v.x
            val translationY = subStep.dt * v.y
            if (translationX * translationX + translationY * translationY > Settings.maxTranslationSquared) {
                val ratio = Settings.maxTranslation / MathUtils.sqrt(translationX * translationX + translationY * translationY)
                v.mulLocal(ratio)
            }
            val rotation = subStep.dt * w
            if (rotation * rotation > Settings.maxRotationSquared) {
                w *= Settings.maxRotation / MathUtils.abs(rotation)
            }
            c.x += subStep.dt * v.x
            c.y += subStep.dt * v.y
            a += subStep.dt * w
            m_positions[index].a = a
            m_velocities[index].w = w
        }

        for (index in 0 until m_bodyCount) {
            val body = m_bodies[index]!!
            if (body.m_type == BodyType.STATIC) {
                m_positions[index].c.set(body.m_sweep.c)
                m_positions[index].a = body.m_sweep.a
                m_velocities[index].v.setZero()
                m_velocities[index].w = 0f
            }
        }

        for (index in 0 until m_bodyCount) {
            val body = m_bodies[index]!!
            body.m_sweep.c.set(m_positions[index].c)
            body.m_sweep.a = m_positions[index].a
            if (body.m_type == BodyType.STATIC) {
                body.m_linearVelocity.setZero()
                body.m_angularVelocity = 0f
            } else {
                body.m_linearVelocity.set(m_velocities[index].v)
                body.m_angularVelocity = m_velocities[index].w
            }
            body.synchronizeTransform()
        }
    }

    private fun report() {
        val listener = listener ?: return
        for (index in 0 until m_contactCount) {
            val contact = m_contacts[index] ?: continue
            val vc = contactSolver.m_contacts[index] ?: continue
            val manifold = vc.getManifold()
            impulse.count = manifold.pointCount
            for (j in 0 until impulse.count) {
                impulse.normalImpulses[j] = manifold.points[j].normalImpulse
                impulse.tangentImpulses[j] = manifold.points[j].tangentImpulse
            }
            listener.postSolve(contact, impulse)
        }
    }

    private fun solveSleep(dt: Float) {
        var minSleepTime = Float.MAX_VALUE
        for (index in 0 until m_bodyCount) {
            val body = m_bodies[index]!!
            if (body.m_type == BodyType.STATIC) {
                continue
            }
            if ((body.m_flags and Body.e_autoSleepFlag) == 0 || body.m_angularVelocity * body.m_angularVelocity > Settings.angularSleepTolerance * Settings.angularSleepTolerance || Vec2.dot(body.m_linearVelocity, body.m_linearVelocity) > Settings.linearSleepTolerance * Settings.linearSleepTolerance) {
                body.m_sleepTime = 0f
                minSleepTime = 0f
            } else {
                body.m_sleepTime += dt
                minSleepTime = MathUtils.min(minSleepTime, body.m_sleepTime)
            }
        }
        if (minSleepTime >= Settings.timeToSleep) {
            for (index in 0 until m_bodyCount) {
                m_bodies[index]?.setAwake(false)
            }
        }
    }
}
