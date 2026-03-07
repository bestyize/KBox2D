package xyz.thewind.kbox2d.dynamics

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.common.Vec2

class WorldStepTest {
    @Test
    fun stepAppliesGravityAndMovesDynamicBody() {
        val world = World(gravity = Vec2(0f, -10f))
        val body = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(0f, 10f)
        })!!
        body.createFixture(FixtureDef().apply {
            shape = CircleShape().apply { m_radius = 1f }
            density = 1f
        })

        val startY = body.getPosition().y
        world.step(1f / 60f, velocityIterations = 8, positionIterations = 3)

        assertTrue(body.getPosition().y < startY)
        assertTrue(body.getLinearVelocity().y < 0f)
    }


    @Test
    fun staticObstacleRemainsFixedAfterRepeatedImpacts() {
        val world = World(gravity = Vec2(0f, -10f))
        val obstacle = world.createBody(BodyDef().apply {
            type = BodyType.STATIC
            position = Vec2(0f, 3f)
            angle = 0.2f
        })!!
        obstacle.createFixture(FixtureDef().apply {
            shape = CircleShape().apply { m_radius = 0.8f }
            density = 0f
        })

        repeat(6) { index ->
            val body = world.createBody(BodyDef().apply {
                type = BodyType.DYNAMIC
                position = Vec2(-1.2f + index * 0.45f, 8f + index)
            })!!
            body.createFixture(FixtureDef().apply {
                shape = CircleShape().apply { m_radius = 0.35f }
                density = 1f
                restitution = 0.55f
            })
        }

        repeat(360) {
            world.step(1f / 60f, 8, 3)
        }

        assertEquals(0f, obstacle.getPosition().x, 0.0001f)
        assertEquals(3f, obstacle.getPosition().y, 0.0001f)
        assertEquals(0.2f, obstacle.getAngle(), 0.0001f)

        val freshBody = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(0f, 10f)
        })!!
        freshBody.createFixture(FixtureDef().apply {
            shape = CircleShape().apply { m_radius = 0.4f }
            density = 1f
        })
        val startY = freshBody.getPosition().y
        repeat(10) {
            world.step(1f / 60f, 8, 3)
        }
        assertTrue(freshBody.getPosition().y < startY)
    }

    @Test
    fun bulletBodyDoesNotTunnelThroughStaticCircle() {
        val world = World(gravity = Vec2(0f, 0f))
        val wall = world.createBody(BodyDef().apply {
            type = BodyType.STATIC
            position = Vec2(0f, 0f)
        })!!
        wall.createFixture(FixtureDef().apply {
            shape = CircleShape().apply { m_radius = 1f }
            density = 0f
        })

        val bullet = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(-10f, 0f)
            linearVelocity = Vec2(120f, 0f)
            bullet = true
        })!!
        bullet.createFixture(FixtureDef().apply {
            shape = CircleShape().apply { m_radius = 0.5f }
            density = 1f
        })

        world.step(0.2f, velocityIterations = 8, positionIterations = 3)

        assertTrue(bullet.getPosition().x < 3f)
        assertTrue(bullet.getLinearVelocity().x < 120f)
    }

}
