package xyz.thewind.kbox2d.dynamics

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Vec2

class BodyRuntimeTest {
    @Test
    fun coordinateTransformsRoundTrip() {
        val body = Body(
            BodyDef().apply {
                type = BodyType.DYNAMIC
                position = Vec2(2f, 3f)
                angle = MathUtils.HALF_PI
            },
        )

        val localPoint = Vec2(1f, 0f)
        val worldPoint = body.getWorldPoint(localPoint)
        assertEquals(2f, worldPoint.x, 1e-5f)
        assertEquals(4f, worldPoint.y, 1e-5f)

        val roundTrip = body.getLocalPoint(worldPoint)
        assertEquals(localPoint.x, roundTrip.x, 1e-5f)
        assertEquals(localPoint.y, roundTrip.y, 1e-5f)
    }

    @Test
    fun resetMassDataUsesFixtureMass() {
        val world = World()
        val body = world.createBody(
            BodyDef().apply {
                type = BodyType.DYNAMIC
            },
        )
        assertNotNull(body)

        body.createFixture(FixtureDef().apply {
            shape = CircleShape().apply {
                m_radius = 2f
                m_p.set(3f, 0f)
            }
            density = 2f
        })

        assertTrue(body.getMass() > 0f)
        assertTrue(body.m_invMass > 0f)
        assertTrue(body.getInertia() > 0f)
        assertEquals(3f, body.getLocalCenter().x, 1e-5f)
        assertEquals(0f, body.getLocalCenter().y, 1e-5f)
    }

    @Test
    fun worldTracksBodyLifecycle() {
        val world = World()
        val bodyA = world.createBody(BodyDef().apply { type = BodyType.DYNAMIC })
        val bodyB = world.createBody(BodyDef().apply { type = BodyType.STATIC })
        assertNotNull(bodyA)
        assertNotNull(bodyB)
        assertEquals(2, world.getBodyCount())
        assertTrue(world.getBodyList() === bodyB)

        world.destroyBody(bodyB)

        assertEquals(1, world.getBodyCount())
        assertTrue(world.getBodyList() === bodyA)
        assertEquals(null, bodyB.m_world)
    }
}
