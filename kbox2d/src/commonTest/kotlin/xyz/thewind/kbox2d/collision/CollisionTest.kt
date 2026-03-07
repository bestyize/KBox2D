package xyz.thewind.kbox2d.collision

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue
import xyz.thewind.kbox2d.collision.Manifold.ManifoldType
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.collision.shapes.EdgeShape
import xyz.thewind.kbox2d.collision.shapes.PolygonShape
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2

class CollisionTest {
    private val collision = Collision()

    @Test
    fun collideCirclesBuildsCircleManifold() {
        val circleA = CircleShape().apply {
            m_p.set(0f, 0f)
            m_radius = 1f
        }
        val circleB = CircleShape().apply {
            m_p.set(0f, 0f)
            m_radius = 1f
        }
        val xfA = Transform().apply { setIdentity() }
        val xfB = Transform().apply {
            setIdentity()
            p.set(1.5f, 0f)
        }
        val manifold = Manifold()

        collision.collideCircles(manifold, circleA, xfA, circleB, xfB)

        assertEquals(1, manifold.pointCount)
        assertEquals(ManifoldType.CIRCLES, manifold.type)
    }

    @Test
    fun collidePolygonAndCircleBuildsFaceContact() {
        val polygon = PolygonShape().apply { setAsBox(1f, 1f) }
        val circle = CircleShape().apply {
            m_p.set(0f, 0f)
            m_radius = 0.5f
        }
        val xfA = Transform().apply { setIdentity() }
        val xfB = Transform().apply {
            setIdentity()
            p.set(1.2f, 0f)
        }
        val manifold = Manifold()

        collision.collidePolygonAndCircle(manifold, polygon, xfA, circle, xfB)

        assertEquals(1, manifold.pointCount)
        assertEquals(ManifoldType.FACE_A, manifold.type)
    }

    @Test
    fun collidePolygonsFindsOverlap() {
        val polyA = PolygonShape().apply { setAsBox(1f, 1f) }
        val polyB = PolygonShape().apply { setAsBox(1f, 1f) }
        val xfA = Transform().apply { setIdentity() }
        val xfB = Transform().apply {
            setIdentity()
            p.set(1f, 0f)
        }
        val manifold = Manifold()

        collision.collidePolygons(manifold, polyA, xfA, polyB, xfB)

        assertTrue(manifold.pointCount > 0)
        assertTrue(manifold.type == ManifoldType.FACE_A || manifold.type == ManifoldType.FACE_B)
    }

    @Test
    fun collideEdgeAndCircleFindsFaceContact() {
        val edge = EdgeShape().apply { set(Vec2(-1f, 0f), Vec2(1f, 0f)) }
        val circle = CircleShape().apply {
            m_p.set(0f, 0f)
            m_radius = 0.5f
        }
        val xfA = Transform().apply { setIdentity() }
        val xfB = Transform().apply {
            setIdentity()
            p.set(0f, 0.2f)
        }
        val manifold = Manifold()

        collision.collideEdgeAndCircle(manifold, edge, xfA, circle, xfB)

        assertEquals(1, manifold.pointCount)
        assertEquals(ManifoldType.FACE_A, manifold.type)
    }
}
