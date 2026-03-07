package xyz.thewind.kbox2d.dynamics

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertSame
import kotlin.test.assertTrue
import xyz.thewind.kbox2d.callbacks.DebugDraw
import xyz.thewind.kbox2d.callbacks.QueryCallback
import xyz.thewind.kbox2d.callbacks.RayCastCallback
import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.shapes.PolygonShape
import xyz.thewind.kbox2d.common.Color3f
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.joints.MouseJoint
import xyz.thewind.kbox2d.dynamics.joints.MouseJointDef
import xyz.thewind.kbox2d.dynamics.joints.PulleyJoint
import xyz.thewind.kbox2d.dynamics.joints.PulleyJointDef

class WorldApiCoverageTest {
    @Test
    fun worldQueryAabbAndRaycastReachFixture() {
        val world = World(gravity = Vec2(0f, 0f))
        val body = world.createBody(BodyDef().apply {
            type = BodyType.STATIC
            position = Vec2(0f, 0f)
        })!!
        val fixture = body.createFixture(FixtureDef().apply {
            shape = PolygonShape().apply { setAsBox(1f, 1f) }
            density = 0f
        })!!

        val queried = mutableListOf<Fixture>()
        world.queryAABB(QueryCallback { candidate ->
            queried += candidate
            true
        }, AABB().apply {
            lowerBound.set(-0.5f, -0.5f)
            upperBound.set(0.5f, 0.5f)
        })

        var hitFixture: Fixture? = null
        var hitFraction = -1f
        world.raycast(RayCastCallback { candidate, _, _, fraction ->
            hitFixture = candidate
            hitFraction = fraction
            0f
        }, Vec2(-2f, 0f), Vec2(2f, 0f))

        assertTrue(queried.contains(fixture))
        assertSame(fixture, hitFixture)
        assertTrue(hitFraction in 0f..1f)
        assertEquals(0.25f, hitFraction, 0.0001f)
    }

    @Test
    fun worldShiftOriginMovesBodiesJointsAndGravityStateRemainsUsable() {
        val world = World(gravity = Vec2(0f, -10f))
        val dynamicBodyA = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(5f, 3f)
        })!!
        val dynamicBodyB = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(8f, 4f)
        })!!
        for (body in listOf(dynamicBodyA, dynamicBodyB)) {
            body.createFixture(FixtureDef().apply {
                shape = PolygonShape().apply { setAsBox(0.5f, 0.5f) }
                density = 1f
            })
        }

        val mouseJoint = world.createJoint(MouseJointDef().apply {
            bodyA = dynamicBodyA
            bodyB = dynamicBodyB
            target.set(9f, 7f)
            maxForce = 10f
        }) as MouseJoint

        val pulleyJoint = world.createJoint(PulleyJointDef().apply {
            initialize(dynamicBodyA, dynamicBodyB, Vec2(4f, 8f), Vec2(10f, 8f), dynamicBodyA.getPosition().clone(), dynamicBodyB.getPosition().clone(), 1f)
        }) as PulleyJoint

        world.setGravity(Vec2(0f, -12f))
        world.shiftOrigin(Vec2(2f, 1f))

        assertEquals(3f, dynamicBodyA.getPosition().x, 0.0001f)
        assertEquals(2f, dynamicBodyA.getPosition().y, 0.0001f)
        assertEquals(6f, dynamicBodyB.getPosition().x, 0.0001f)
        assertEquals(3f, dynamicBodyB.getPosition().y, 0.0001f)
        assertEquals(7f, mouseJoint.getTarget().x, 0.0001f)
        assertEquals(6f, mouseJoint.getTarget().y, 0.0001f)
        assertEquals(2f, pulleyJoint.getGroundAnchorA().x, 0.0001f)
        assertEquals(7f, pulleyJoint.getGroundAnchorA().y, 0.0001f)
        assertEquals(8f, pulleyJoint.getGroundAnchorB().x, 0.0001f)
        assertEquals(7f, pulleyJoint.getGroundAnchorB().y, 0.0001f)
        assertEquals(-12f, world.getGravity().y, 0.0001f)

        val queried = mutableListOf<Fixture>()
        world.queryAABB(QueryCallback {
            queried += it
            true
        }, AABB().apply {
            lowerBound.set(2.4f, 1.4f)
            upperBound.set(3.6f, 2.6f)
        })
        assertTrue(queried.isNotEmpty())
    }

    @Test
    fun worldDumpReturnsUsefulSummary() {
        val world = World(gravity = Vec2(0f, -10f))
        world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(0f, 1f)
        })!!.createFixture(FixtureDef().apply {
            shape = PolygonShape().apply { setAsBox(0.5f, 0.5f) }
            density = 1f
        })

        val dump = world.dump()

        assertTrue(dump.contains("World dump:"))
        assertTrue(dump.contains("gravity="))
        assertTrue(dump.contains("bodies=1"))
        assertTrue(dump.contains("Profile:"))
    }

    @Test
    fun worldExposesDebugDrawProfileAndCounts() {
        val world = World(gravity = Vec2(0f, 0f))
        val body = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(1f, 2f)
        })!!
        body.createFixture(FixtureDef().apply {
            shape = PolygonShape().apply { setAsBox(0.5f, 0.5f) }
            density = 1f
        })

        val debugDraw = CountingDebugDraw()
        debugDraw.setFlags(DebugDraw.e_shapeBit or DebugDraw.e_aabbBit or DebugDraw.e_centerOfMassBit)
        world.setDebugDraw(debugDraw)
        world.drawDebugData()

        assertSame(debugDraw, world.getDebugDraw())
        assertSame(world.m_profile, world.getProfile())
        assertEquals(1, world.getBodyCount())
        assertEquals(1, world.getProxyCount())
        assertNotNull(world.getBodyList())
        assertTrue(debugDraw.solidPolygonCount > 0)
        assertTrue(debugDraw.segmentCount > 0)
        assertTrue(debugDraw.transformCount > 0)
    }

    private class CountingDebugDraw : DebugDraw() {
        var solidPolygonCount: Int = 0
        var segmentCount: Int = 0
        var transformCount: Int = 0

        override fun drawPoint(argPoint: Vec2, argRadiusOnScreen: Float, argColor: Color3f) = Unit

        override fun drawSolidPolygon(vertices: Array<Vec2>, vertexCount: Int, color: Color3f) {
            solidPolygonCount++
        }

        override fun drawCircle(center: Vec2, radius: Float, color: Color3f) = Unit

        override fun drawSolidCircle(center: Vec2, radius: Float, axis: Vec2, color: Color3f) = Unit

        override fun drawSegment(p1: Vec2, p2: Vec2, color: Color3f) {
            segmentCount++
        }

        override fun drawTransform(xf: Transform) {
            transformCount++
        }

        override fun drawString(x: Float, y: Float, s: String, color: Color3f) = Unit

        override fun drawParticles(centers: Array<Vec2>, radius: Float, colors: Array<xyz.thewind.kbox2d.particle.ParticleColor>?, count: Int) = Unit

        override fun drawParticlesWireframe(centers: Array<Vec2>, radius: Float, colors: Array<xyz.thewind.kbox2d.particle.ParticleColor>?, count: Int) = Unit
    }
}
