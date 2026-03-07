package xyz.thewind.kbox2d.common

import kotlin.math.roundToInt
import kotlin.random.Random
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse

class MathTest {
    private val max = (Float.MAX_VALUE / 1000f).toInt()
    private val randomIterations = 100

    @Test
    fun fastMathMatchesStdlib() {
        val random = Random(42)
        repeat(randomIterations) {
            val value = random.nextFloat() * max - max / 2f
            assertEquals(kotlin.math.floor(value).toInt(), MathUtils.floor(value))
            assertEquals(kotlin.math.ceil(value).toInt(), MathUtils.ceil(value))
            assertEquals(value.roundToInt(), MathUtils.round(value))
            assertEquals(kotlin.math.abs(value), MathUtils.abs(value))
        }

        repeat(randomIterations) {
            val a = random.nextFloat() * max - max / 2f
            val b = random.nextFloat() * max - max / 2f
            assertEquals(kotlin.math.max(a, b), MathUtils.max(a, b))
            assertEquals(kotlin.math.min(a, b), MathUtils.min(a, b))
        }
    }

    @Test
    fun vec2OperationsStayCompatible() {
        val vector = Vec2().apply {
            x = 0f
            y = 1f
        }
        vector.subLocal(Vec2(10f, 10f))
        assertEquals(-10f, vector.x)
        assertEquals(-9f, vector.y)

        val other = vector.add(Vec2(1f, 1f))
        assertEquals(-9f, other.x)
        assertEquals(-8f, other.y)
        assertFalse(vector == other)
    }

    @Test
    fun mat22UnsafeAndSafeVariantsMatch() {
        val vector = Vec2(10f, -1.3f)
        val m1 = Mat22(1f, 34f, -3f, 3f)
        val m2 = Mat22(2f, -1f, 4.1f, -4f)
        val vectorOut = Vec2()
        val matrixOut = Mat22()

        Mat22.mulToOutUnsafe(m1, m2, matrixOut)
        assertEquals(Mat22.mul(m1, m2), matrixOut)

        Mat22.mulToOutUnsafe(m1, vector, vectorOut)
        assertEquals(Mat22.mul(m1, vector), vectorOut)

        Mat22.mulTransToOutUnsafe(m1, m2, matrixOut)
        assertEquals(Mat22.mulTrans(m1, m2), matrixOut)

        Mat22.mulTransToOutUnsafe(m1, vector, vectorOut)
        assertEquals(Mat22.mulTrans(m1, vector), vectorOut)
    }

    @Test
    fun mat33SolveMatchesUpstreamExpectation() {
        val mat = Mat33()
        mat.ex.set(3f, 19f, -5f)
        mat.ey.set(-1f, 1f, 4f)
        mat.ez.set(-10f, 4f, 4f)

        val b = Vec3(4f, 1f, 2f)
        assertEquals(Vec3(0.096f, 1.1013334f, -0.48133332f), mat.solve33(b))

        val b2 = Vec2(4f, 1f)
        assertEquals(Vec2(0.22727273f, -3.318182f), mat.solve22(b2))
    }

    @Test
    fun vec3CrossMatchesUpstreamExpectation() {
        val v1 = Vec3()
        val v2 = Vec3()
        assertEquals(Vec3(1f, -15f, 36f), Vec3.cross(v1.set(9f, 3f, 1f), v2.set(3f, 5f, 2f)))
    }
}
