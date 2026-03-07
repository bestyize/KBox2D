package xyz.thewind.kbox2d.dynamics.contacts

import xyz.thewind.kbox2d.collision.Manifold
import xyz.thewind.kbox2d.collision.WorldManifold
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body
import xyz.thewind.kbox2d.dynamics.TimeStep

class ContactSolver {
    var m_step: TimeStep = TimeStep()
    var m_positions: Array<Position> = emptyArray()
    var m_velocities: Array<Velocity> = emptyArray()
    var m_contacts: Array<Contact?> = emptyArray()
    var m_count: Int = 0

    private var m_positionConstraints: Array<ContactPositionConstraint> = emptyArray()
    private var m_velocityConstraints: Array<ContactVelocityConstraint> = emptyArray()
    private val worldManifold = WorldManifold()
    private val xfA = Transform()
    private val xfB = Transform()
    private val tangent = Vec2()
    private val p = Vec2()

    fun init(def: ContactSolverDef) {
        m_step = def.step
        m_positions = def.positions
        m_velocities = def.velocities
        m_contacts = def.contacts
        m_count = def.count

        if (m_positionConstraints.size < m_count) {
            m_positionConstraints = Array(m_count) { ContactPositionConstraint() }
        }
        if (m_velocityConstraints.size < m_count) {
            m_velocityConstraints = Array(m_count) { ContactVelocityConstraint() }
        }

        for (index in 0 until m_count) {
            val contact = m_contacts[index]!!
            val fixtureA = contact.getFixtureA()
            val fixtureB = contact.getFixtureB()
            val shapeA = fixtureA.getShape()
            val shapeB = fixtureB.getShape()
            val bodyA = fixtureA.getBody()!!
            val bodyB = fixtureB.getBody()!!
            val manifold = contact.getManifold()
            val pointCount = manifold.pointCount
            require(pointCount > 0)

            val vc = m_velocityConstraints[index]
            vc.friction = contact.getFriction()
            vc.restitution = contact.getRestitution()
            vc.tangentSpeed = contact.getTangentSpeed()
            vc.indexA = bodyA.m_islandIndex
            vc.indexB = bodyB.m_islandIndex
            vc.invMassA = bodyA.m_invMass
            vc.invMassB = bodyB.m_invMass
            vc.invIA = bodyA.m_invI
            vc.invIB = bodyB.m_invI
            vc.contactIndex = index
            vc.pointCount = pointCount
            vc.K.setZero()
            vc.normalMass.setZero()

            val pc = m_positionConstraints[index]
            pc.indexA = bodyA.m_islandIndex
            pc.indexB = bodyB.m_islandIndex
            pc.invMassA = bodyA.m_invMass
            pc.invMassB = bodyB.m_invMass
            pc.localCenterA.set(bodyA.m_sweep.localCenter)
            pc.localCenterB.set(bodyB.m_sweep.localCenter)
            pc.invIA = bodyA.m_invI
            pc.invIB = bodyB.m_invI
            pc.localNormal.set(manifold.localNormal)
            pc.localPoint.set(manifold.localPoint)
            pc.pointCount = pointCount
            pc.radiusA = shapeA.m_radius
            pc.radiusB = shapeB.m_radius
            pc.type = manifold.type

            worldManifold.initialize(manifold, bodyA.m_xf, shapeA.m_radius, bodyB.m_xf, shapeB.m_radius)
            vc.normal.set(worldManifold.normal)

            for (j in 0 until pointCount) {
                val cp = manifold.points[j]
                val vcp = vc.points[j]
                if (m_step.warmStarting) {
                    vcp.normalImpulse = m_step.dtRatio * cp.normalImpulse
                    vcp.tangentImpulse = m_step.dtRatio * cp.tangentImpulse
                } else {
                    vcp.normalImpulse = 0f
                    vcp.tangentImpulse = 0f
                }
                vcp.rA.set(worldManifold.points[j]).subLocal(bodyA.m_sweep.c)
                vcp.rB.set(worldManifold.points[j]).subLocal(bodyB.m_sweep.c)

                val rnA = Vec2.cross(vcp.rA, vc.normal)
                val rnB = Vec2.cross(vcp.rB, vc.normal)
                val kNormal = vc.invMassA + vc.invMassB + vc.invIA * rnA * rnA + vc.invIB * rnB * rnB
                vcp.normalMass = if (kNormal > 0f) 1f / kNormal else 0f

                tangent.x = vc.normal.y
                tangent.y = -vc.normal.x
                val rtA = Vec2.cross(vcp.rA, tangent)
                val rtB = Vec2.cross(vcp.rB, tangent)
                val kTangent = vc.invMassA + vc.invMassB + vc.invIA * rtA * rtA + vc.invIB * rtB * rtB
                vcp.tangentMass = if (kTangent > 0f) 1f / kTangent else 0f
                vcp.velocityBias = 0f

                val dvx = bodyB.m_linearVelocity.x - bodyB.m_angularVelocity * vcp.rB.y - bodyA.m_linearVelocity.x + bodyA.m_angularVelocity * vcp.rA.y
                val dvy = bodyB.m_linearVelocity.y + bodyB.m_angularVelocity * vcp.rB.x - bodyA.m_linearVelocity.y - bodyA.m_angularVelocity * vcp.rA.x
                val vRel = vc.normal.x * dvx + vc.normal.y * dvy
                if (vRel < -Settings.velocityThreshold) {
                    vcp.velocityBias = -vc.restitution * vRel
                }

                pc.localPoints[j].set(cp.localPoint)
            }
        }
    }

    fun warmStart() {
        for (index in 0 until m_count) {
            val vc = m_velocityConstraints[index]
            val indexA = vc.indexA
            val indexB = vc.indexB
            val mA = vc.invMassA
            val iA = vc.invIA
            val mB = vc.invMassB
            val iB = vc.invIB
            val normal = vc.normal
            tangent.x = normal.y
            tangent.y = -normal.x
            val vA = m_velocities[indexA].v
            var wA = m_velocities[indexA].w
            val vB = m_velocities[indexB].v
            var wB = m_velocities[indexB].w

            for (j in 0 until vc.pointCount) {
                val vcp = vc.points[j]
                p.x = normal.x * vcp.normalImpulse + tangent.x * vcp.tangentImpulse
                p.y = normal.y * vcp.normalImpulse + tangent.y * vcp.tangentImpulse
                vA.x -= mA * p.x
                vA.y -= mA * p.y
                wA -= iA * Vec2.cross(vcp.rA, p)
                vB.x += mB * p.x
                vB.y += mB * p.y
                wB += iB * Vec2.cross(vcp.rB, p)
            }

            m_velocities[indexA].w = wA
            m_velocities[indexB].w = wB
        }
    }

    fun solveVelocityConstraints() {
        for (index in 0 until m_count) {
            val vc = m_velocityConstraints[index]
            val indexA = vc.indexA
            val indexB = vc.indexB
            val mA = vc.invMassA
            val iA = vc.invIA
            val mB = vc.invMassB
            val iB = vc.invIB
            val normal = vc.normal
            tangent.x = normal.y
            tangent.y = -normal.x
            val vA = m_velocities[indexA].v
            var wA = m_velocities[indexA].w
            val vB = m_velocities[indexB].v
            var wB = m_velocities[indexB].w

            for (j in 0 until vc.pointCount) {
                val vcp = vc.points[j]
                val dvx = vB.x - wB * vcp.rB.y - vA.x + wA * vcp.rA.y
                val dvy = vB.y + wB * vcp.rB.x - vA.y - wA * vcp.rA.x
                val vt = Vec2.dot(Vec2(dvx, dvy), tangent) - vc.tangentSpeed
                var lambda = vcp.tangentMass * -vt
                val maxFriction = vc.friction * vcp.normalImpulse
                val newImpulse = MathUtils.clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction)
                lambda = newImpulse - vcp.tangentImpulse
                vcp.tangentImpulse = newImpulse
                p.x = tangent.x * lambda
                p.y = tangent.y * lambda
                vA.x -= mA * p.x
                vA.y -= mA * p.y
                wA -= iA * Vec2.cross(vcp.rA, p)
                vB.x += mB * p.x
                vB.y += mB * p.y
                wB += iB * Vec2.cross(vcp.rB, p)
            }

            for (j in 0 until vc.pointCount) {
                val vcp = vc.points[j]
                val dvx = vB.x - wB * vcp.rB.y - vA.x + wA * vcp.rA.y
                val dvy = vB.y + wB * vcp.rB.x - vA.y - wA * vcp.rA.x
                val vn = dvx * normal.x + dvy * normal.y
                var lambda = -vcp.normalMass * (vn - vcp.velocityBias)
                val newImpulse = MathUtils.max(vcp.normalImpulse + lambda, 0f)
                lambda = newImpulse - vcp.normalImpulse
                vcp.normalImpulse = newImpulse
                p.x = normal.x * lambda
                p.y = normal.y * lambda
                vA.x -= mA * p.x
                vA.y -= mA * p.y
                wA -= iA * Vec2.cross(vcp.rA, p)
                vB.x += mB * p.x
                vB.y += mB * p.y
                wB += iB * Vec2.cross(vcp.rB, p)
            }

            m_velocities[indexA].w = wA
            m_velocities[indexB].w = wB
        }
    }

    fun storeImpulses() {
        for (index in 0 until m_count) {
            val vc = m_velocityConstraints[index]
            val manifold = m_contacts[index]!!.getManifold()
            for (j in 0 until vc.pointCount) {
                manifold.points[j].normalImpulse = vc.points[j].normalImpulse
                manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse
            }
        }
    }

    fun solvePositionConstraints(): Boolean {
        var minSeparation = 0f
        val psm = PositionSolverManifold()
        for (index in 0 until m_count) {
            val pc = m_positionConstraints[index]
            val indexA = pc.indexA
            val indexB = pc.indexB
            val mA = pc.invMassA
            val iA = pc.invIA
            val mB = pc.invMassB
            val iB = pc.invIB
            val localCenterA = pc.localCenterA
            val localCenterB = pc.localCenterB
            var cA = m_positions[indexA].c
            var aA = m_positions[indexA].a
            var cB = m_positions[indexB].c
            var aB = m_positions[indexB].a

            for (j in 0 until pc.pointCount) {
                xfA.q.set(aA)
                xfB.q.set(aB)
                Rot.mulToOutUnsafe(xfA.q, localCenterA, xfA.p)
                Rot.mulToOutUnsafe(xfB.q, localCenterB, xfB.p)
                xfA.p.negateLocal().addLocal(cA)
                xfB.p.negateLocal().addLocal(cB)

                psm.initialize(pc, xfA, xfB, j)
                val normal = psm.normal
                val point = psm.point
                val separation = psm.separation
                minSeparation = MathUtils.min(minSeparation, separation)

                val rA = point.sub(cA)
                val rB = point.sub(cB)
                val c = MathUtils.clamp(Settings.baumgarte * (separation + Settings.linearSlop), -Settings.maxLinearCorrection, 0f)
                val rnA = Vec2.cross(rA, normal)
                val rnB = Vec2.cross(rB, normal)
                val k = mA + mB + iA * rnA * rnA + iB * rnB * rnB
                val impulse = if (k > 0f) -c / k else 0f
                p.x = normal.x * impulse
                p.y = normal.y * impulse
                cA.x -= mA * p.x
                cA.y -= mA * p.y
                aA -= iA * Vec2.cross(rA, p)
                cB.x += mB * p.x
                cB.y += mB * p.y
                aB += iB * Vec2.cross(rB, p)
            }

            m_positions[indexA].a = aA
            m_positions[indexB].a = aB
        }
        return minSeparation >= -3f * Settings.linearSlop
    }

    fun solveTOIPositionConstraints(toiIndexA: Int, toiIndexB: Int): Boolean {
        var minSeparation = 0f
        val psm = PositionSolverManifold()
        for (index in 0 until m_count) {
            val pc = m_positionConstraints[index]
            val indexA = pc.indexA
            val indexB = pc.indexB
            val localCenterA = pc.localCenterA
            val localCenterB = pc.localCenterB
            val mA = if (indexA == toiIndexA || indexA == toiIndexB) pc.invMassA else 0f
            val iA = if (indexA == toiIndexA || indexA == toiIndexB) pc.invIA else 0f
            val mB = if (indexB == toiIndexA || indexB == toiIndexB) pc.invMassB else 0f
            val iB = if (indexB == toiIndexA || indexB == toiIndexB) pc.invIB else 0f
            var cA = m_positions[indexA].c
            var aA = m_positions[indexA].a
            var cB = m_positions[indexB].c
            var aB = m_positions[indexB].a

            for (j in 0 until pc.pointCount) {
                xfA.q.set(aA)
                xfB.q.set(aB)
                Rot.mulToOutUnsafe(xfA.q, localCenterA, xfA.p)
                Rot.mulToOutUnsafe(xfB.q, localCenterB, xfB.p)
                xfA.p.negateLocal().addLocal(cA)
                xfB.p.negateLocal().addLocal(cB)

                psm.initialize(pc, xfA, xfB, j)
                val normal = psm.normal
                val point = psm.point
                val separation = psm.separation
                minSeparation = MathUtils.min(minSeparation, separation)

                val rA = point.sub(cA)
                val rB = point.sub(cB)
                val c = MathUtils.clamp(
                    Settings.toiBaugarte * (separation + Settings.linearSlop),
                    -Settings.maxLinearCorrection,
                    0f,
                )
                val rnA = Vec2.cross(rA, normal)
                val rnB = Vec2.cross(rB, normal)
                val k = mA + mB + iA * rnA * rnA + iB * rnB * rnB
                val impulse = if (k > 0f) -c / k else 0f
                p.x = normal.x * impulse
                p.y = normal.y * impulse
                cA.x -= mA * p.x
                cA.y -= mA * p.y
                aA -= iA * Vec2.cross(rA, p)
                cB.x += mB * p.x
                cB.y += mB * p.y
                aB += iB * Vec2.cross(rB, p)
            }

            m_positions[indexA].a = aA
            m_positions[indexB].a = aB
        }
        return minSeparation >= -1.5f * Settings.linearSlop
    }


    class ContactSolverDef {
        var step: TimeStep = TimeStep()
        var contacts: Array<Contact?> = emptyArray()
        var count: Int = 0
        var positions: Array<Position> = emptyArray()
        var velocities: Array<Velocity> = emptyArray()
    }

    private class PositionSolverManifold {
        val normal: Vec2 = Vec2()
        val point: Vec2 = Vec2()
        var separation: Float = 0f

        fun initialize(pc: ContactPositionConstraint, xfA: Transform, xfB: Transform, index: Int) {
            when (pc.type) {
                Manifold.ManifoldType.CIRCLES -> {
                    val pointA = Transform.mul(xfA, pc.localPoint)
                    val pointB = Transform.mul(xfB, pc.localPoints[0])
                    normal.set(pointB).subLocal(pointA)
                    if (normal.lengthSquared() > Settings.EPSILON * Settings.EPSILON) {
                        normal.normalize()
                    } else {
                        normal.set(1f, 0f)
                    }
                    point.x = (pointA.x + pointB.x) * 0.5f
                    point.y = (pointA.y + pointB.y) * 0.5f
                    separation = Vec2.dot(pointB.sub(pointA), normal) - pc.radiusA - pc.radiusB
                }
                Manifold.ManifoldType.FACE_A -> {
                    Rot.mulToOutUnsafe(xfA.q, pc.localNormal, normal)
                    val planePoint = Transform.mul(xfA, pc.localPoint)
                    val clipPoint = Transform.mul(xfB, pc.localPoints[index])
                    separation = Vec2.dot(clipPoint.sub(planePoint), normal) - pc.radiusA - pc.radiusB
                    point.set(clipPoint)
                }
                Manifold.ManifoldType.FACE_B -> {
                    Rot.mulToOutUnsafe(xfB.q, pc.localNormal, normal)
                    val planePoint = Transform.mul(xfB, pc.localPoint)
                    val clipPoint = Transform.mul(xfA, pc.localPoints[index])
                    separation = Vec2.dot(clipPoint.sub(planePoint), normal) - pc.radiusA - pc.radiusB
                    point.set(clipPoint)
                    normal.negateLocal()
                }
                null -> {
                    normal.set(1f, 0f)
                    point.setZero()
                    separation = 0f
                }
            }
        }
    }
}
