package xyz.thewind.kbox2d.collision.broadphase

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue
import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.common.Vec2

class BroadPhaseTest {
    @Test
    fun updatePairsFindsOverlappingProxies() {
        val broadPhase = DefaultBroadPhaseBuffer(DynamicTree())
        val pairs = mutableListOf<Pair<String, String>>()

        broadPhase.createProxy(AABB(Vec2(0f, 0f), Vec2(1f, 1f)), "A")
        broadPhase.createProxy(AABB(Vec2(0.5f, 0.5f), Vec2(1.5f, 1.5f)), "B")
        broadPhase.createProxy(AABB(Vec2(5f, 5f), Vec2(6f, 6f)), "C")

        broadPhase.updatePairs { userDataA, userDataB ->
            pairs += (userDataA as String) to (userDataB as String)
        }

        assertEquals(1, pairs.size)
        assertTrue(setOf(pairs.first().first, pairs.first().second) == setOf("A", "B"))
    }

    @Test
    fun queryVisitsOnlyIntersectingLeaves() {
        val tree = DynamicTree()
        val hit = mutableListOf<String>()

        tree.createProxy(AABB(Vec2(-1f, -1f), Vec2(1f, 1f)), "origin")
        tree.createProxy(AABB(Vec2(10f, 10f), Vec2(11f, 11f)), "far")

        tree.query({ proxyId ->
            hit += tree.getUserData(proxyId) as String
            true
        }, AABB(Vec2(-0.5f, -0.5f), Vec2(0.5f, 0.5f)))

        assertEquals(listOf("origin"), hit)
    }
}
