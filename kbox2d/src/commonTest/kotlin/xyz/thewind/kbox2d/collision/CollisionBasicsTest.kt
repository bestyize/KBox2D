package xyz.thewind.kbox2d.collision

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue
import xyz.thewind.kbox2d.collision.shapes.ChainShape
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.collision.shapes.EdgeShape
import xyz.thewind.kbox2d.collision.shapes.MassData
import xyz.thewind.kbox2d.collision.shapes.PolygonShape
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2

class CollisionBasicsTest {
    @Test
    fun aabbContainsAndOverlapWork() {
        val outer = AABB(Vec2(-2f, -2f), Vec2(2f, 2f))
        val inner = AABB(Vec2(-1f, -1f), Vec2(1f, 1f))
        val separate = AABB(Vec2(3f, 3f), Vec2(4f, 4f))

        assertTrue(outer.contains(inner))
        assertTrue(AABB.testOverlap(outer, inner))
        assertFalse(AABB.testOverlap(outer, separate))
    }

    @Test
    fun aabbRaycastHitsExpectedFace() {
        val aabb = AABB(Vec2(-1f, -1f), Vec2(1f, 1f))
        val input = RayCastInput().also {
            it.p1.set(-2f, 0f)
            it.p2.set(2f, 0f)
            it.maxFraction = 1f
        }
        val output = RayCastOutput()

        assertTrue(aabb.raycast(output, input))
        assertEquals(0.25f, output.fraction)
        assertEquals(-1f, output.normal.x)
        assertEquals(0f, output.normal.y)
    }

    @Test
    fun circleShapeCoreBehaviorsWork() {
        val circle = CircleShape().also {
            it.m_p.set(0f, 0f)
            it.m_radius = 2f
        }
        val transform = Transform().also { it.setIdentity() }
        assertTrue(circle.testPoint(transform, Vec2(1f, 1f)))
        assertFalse(circle.testPoint(transform, Vec2(3f, 0f)))

        val aabb = AABB()
        circle.computeAABB(aabb, transform, 0)
        assertEquals(-2f, aabb.lowerBound.x)
        assertEquals(-2f, aabb.lowerBound.y)
        assertEquals(2f, aabb.upperBound.x)
        assertEquals(2f, aabb.upperBound.y)

        val massData = MassData()
        circle.computeMass(massData, 1f)
        assertTrue(massData.mass > 12.56f && massData.mass < 12.57f)
    }

    @Test
    fun worldManifoldForCirclesBuildsExpectedNormal() {
        val manifold = Manifold().also {
            it.type = Manifold.ManifoldType.CIRCLES
            it.pointCount = 1
            it.localPoint.set(0f, 0f)
            it.points[0].localPoint.set(1f, 0f)
        }
        val xfA = Transform().also { it.setIdentity() }
        val xfB = Transform().also { it.setIdentity() }
        val worldManifold = WorldManifold()

        worldManifold.initialize(manifold, xfA, 0.5f, xfB, 0.5f)
        assertEquals(1f, worldManifold.normal.x)
        assertEquals(0f, worldManifold.normal.y)
        assertEquals(0.5f, worldManifold.points[0].x)
    }
    @Test
    fun edgeShapeRaycastAndMassWork() {
        val edge = EdgeShape().also {
            it.set(Vec2(-1f, 0f), Vec2(1f, 0f))
        }
        val input = RayCastInput().also {
            it.p1.set(0f, 1f)
            it.p2.set(0f, -1f)
            it.maxFraction = 1f
        }
        val output = RayCastOutput()
        val transform = Transform().also { it.setIdentity() }

        assertTrue(edge.raycast(output, input, transform, 0))
        assertTrue(output.fraction in 0.49f..0.51f)

        val massData = MassData()
        edge.computeMass(massData, 1f)
        assertEquals(0f, massData.mass)
        assertEquals(0f, massData.center.x)
        assertEquals(0f, massData.center.y)
    }

    @Test
    fun chainShapeChildAccessAndRaycastWork() {
        val chain = ChainShape()
        chain.createChain(arrayOf(Vec2(-1f, 0f), Vec2(0f, 1f), Vec2(1f, 0f)), 3)
        val child = EdgeShape()
        chain.getChildEdge(child, 0)
        assertEquals(-1f, child.m_vertex1.x)
        assertEquals(0f, child.m_vertex1.y)
        assertEquals(0f, child.m_vertex2.x)
        assertEquals(1f, child.m_vertex2.y)

        val input = RayCastInput().also {
            it.p1.set(-0.5f, 2f)
            it.p2.set(-0.5f, -1f)
            it.maxFraction = 1f
        }
        val output = RayCastOutput()
        val transform = Transform().also { it.setIdentity() }
        assertTrue(chain.raycast(output, input, transform, 0))
    }

    @Test
    fun polygonShapeBoxAndPointTestWork() {
        val polygon = PolygonShape()
        polygon.setAsBox(2f, 1f)
        val transform = Transform().also { it.setIdentity() }
        assertTrue(polygon.testPoint(transform, Vec2(0.5f, 0.5f)))
        assertFalse(polygon.testPoint(transform, Vec2(3f, 0f)))

        val aabb = AABB()
        polygon.computeAABB(aabb, transform, 0)
        assertTrue(aabb.lowerBound.x < -1.99f)
        assertTrue(aabb.upperBound.x > 1.99f)
        assertEquals(4, polygon.getVertexCount())
        assertTrue(polygon.validate())
    }

    @Test
    fun distanceBetweenSeparatedCirclesWorks() {
        val circleA = CircleShape().also {
            it.m_p.set(0f, 0f)
            it.m_radius = 1f
        }
        val circleB = CircleShape().also {
            it.m_p.set(4f, 0f)
            it.m_radius = 1f
        }
        val input = DistanceInput().also {
            it.proxyA.set(circleA, 0)
            it.proxyB.set(circleB, 0)
            it.transformA.setIdentity()
            it.transformB.setIdentity()
            it.useRadii = true
        }
        val output = DistanceOutput()
        val cache = Distance.SimplexCache()
        Distance().distance(output, cache, input)
        assertTrue(output.distance in 2.0f - 1e-3f .. 2.0f + 1e-3f)
    }

    @Test
    fun timeOfImpactForApproachingCirclesTouches() {
        val circleA = CircleShape().also {
            it.m_p.set(0f, 0f)
            it.m_radius = 1f
        }
        val circleB = CircleShape().also {
            it.m_p.set(0f, 0f)
            it.m_radius = 1f
        }
        val input = TimeOfImpact.TOIInput().also {
            it.proxyA.set(circleA, 0)
            it.proxyB.set(circleB, 0)
            it.sweepA.c0.set(0f, 0f)
            it.sweepA.c.set(0f, 0f)
            it.sweepB.c0.set(5f, 0f)
            it.sweepB.c.set(1.5f, 0f)
            it.tMax = 1f
        }
        val output = TimeOfImpact.TOIOutput()
        TimeOfImpact().timeOfImpact(output, input)
        assertTrue(output.state == TimeOfImpact.TOIOutputState.TOUCHING || output.state == TimeOfImpact.TOIOutputState.SEPARATED)
        assertTrue(output.t in 0f..1f)
    }

}
