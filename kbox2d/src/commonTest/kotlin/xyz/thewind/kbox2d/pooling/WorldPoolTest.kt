package xyz.thewind.kbox2d.pooling

import kotlin.test.Test
import kotlin.test.assertSame
import kotlin.test.assertTrue
import xyz.thewind.kbox2d.pooling.normal.DefaultWorldPool

class WorldPoolTest {
    @Test
    fun defaultWorldPoolProvidesReusableBuffers() {
        val pool = DefaultWorldPool(8, 4)

        val vecs = pool.popVec2(2)
        vecs[0].set(1f, 2f)
        pool.pushVec2(2)

        val floatArrayA = pool.getFloatArray(4)
        val floatArrayB = pool.getFloatArray(4)
        val intArrayA = pool.getIntArray(3)
        val intArrayB = pool.getIntArray(3)
        val vecArrayA = pool.getVec2Array(2)
        val vecArrayB = pool.getVec2Array(2)

        assertTrue(vecs[0].x == 1f && vecs[0].y == 2f)
        assertSame(floatArrayA, floatArrayB)
        assertSame(intArrayA, intArrayB)
        assertSame(vecArrayA, vecArrayB)
    }
}
