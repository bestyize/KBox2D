package xyz.thewind.kbox2d.collision

import xyz.thewind.kbox2d.collision.Distance.DistanceProxy
import xyz.thewind.kbox2d.collision.Distance.SimplexCache
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Sweep
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2

class TimeOfImpact {
    class TOIInput {
        val proxyA: DistanceProxy = DistanceProxy()
        val proxyB: DistanceProxy = DistanceProxy()
        val sweepA: Sweep = Sweep()
        val sweepB: Sweep = Sweep()
        var tMax: Float = 0f
    }

    enum class TOIOutputState {
        UNKNOWN,
        FAILED,
        OVERLAPPED,
        TOUCHING,
        SEPARATED,
    }

    class TOIOutput {
        var state: TOIOutputState = TOIOutputState.UNKNOWN
        var t: Float = 0f
    }

    private val cache = SimplexCache()
    private val distanceInput = DistanceInput()
    private val xfA = Transform()
    private val xfB = Transform()
    private val distanceOutput = DistanceOutput()
    private val fcn = SeparationFunction()
    private val indexes = kotlin.IntArray(2)
    private val sweepA = Sweep()
    private val sweepB = Sweep()
    private val distance = Distance()

    fun timeOfImpact(output: TOIOutput, input: TOIInput) {
        toiCalls++
        output.state = TOIOutputState.UNKNOWN
        output.t = input.tMax

        val proxyA = input.proxyA
        val proxyB = input.proxyB
        sweepA.set(input.sweepA)
        sweepB.set(input.sweepB)
        sweepA.normalize()
        sweepB.normalize()

        val tMax = input.tMax
        val totalRadius = proxyA.m_radius + proxyB.m_radius
        val target = MathUtils.max(Settings.linearSlop, totalRadius - 3.0f * Settings.linearSlop)
        val tolerance = 0.25f * Settings.linearSlop

        var t1 = 0f
        var iter = 0
        cache.count = 0
        distanceInput.proxyA = input.proxyA
        distanceInput.proxyB = input.proxyB
        distanceInput.useRadii = false

        while (true) {
            sweepA.getTransform(xfA, t1)
            sweepB.getTransform(xfB, t1)
            distanceInput.transformA = xfA
            distanceInput.transformB = xfB
            distance.distance(distanceOutput, cache, distanceInput)

            if (distanceOutput.distance <= 0f) {
                output.state = TOIOutputState.OVERLAPPED
                output.t = 0f
                break
            }
            if (distanceOutput.distance < target + tolerance) {
                output.state = TOIOutputState.TOUCHING
                output.t = t1
                break
            }

            fcn.initialize(cache, proxyA, sweepA, proxyB, sweepB, t1)
            var done = false
            var t2 = tMax
            var pushBackIter = 0
            while (true) {
                val s2 = fcn.findMinSeparation(indexes, t2)
                if (s2 > target + tolerance) {
                    output.state = TOIOutputState.SEPARATED
                    output.t = tMax
                    done = true
                    break
                }
                if (s2 > target - tolerance) {
                    t1 = t2
                    break
                }
                var s1 = fcn.evaluate(indexes[0], indexes[1], t1)
                if (s1 < target - tolerance) {
                    output.state = TOIOutputState.FAILED
                    output.t = t1
                    done = true
                    break
                }
                if (s1 <= target + tolerance) {
                    output.state = TOIOutputState.TOUCHING
                    output.t = t1
                    done = true
                    break
                }

                var rootIterCount = 0
                var a1 = t1
                var a2 = t2
                while (true) {
                    val t = if ((rootIterCount and 1) == 1) {
                        a1 + (target - s1) * (a2 - a1) / (s2 - s1)
                    } else {
                        0.5f * (a1 + a2)
                    }
                    ++rootIterCount
                    ++toiRootIters
                    val s = fcn.evaluate(indexes[0], indexes[1], t)
                    if (MathUtils.abs(s - target) < tolerance) {
                        t2 = t
                        break
                    }
                    if (s > target) {
                        a1 = t
                        s1 = s
                    } else {
                        a2 = t
                    }
                    if (rootIterCount == MAX_ROOT_ITERATIONS) {
                        break
                    }
                }

                toiMaxRootIters = MathUtils.max(toiMaxRootIters, rootIterCount)
                ++pushBackIter
                if (pushBackIter == Settings.maxPolygonVertices || rootIterCount == MAX_ROOT_ITERATIONS) {
                    break
                }
            }

            ++iter
            ++toiIters
            if (done) break
            if (iter == MAX_ITERATIONS) {
                output.state = TOIOutputState.FAILED
                output.t = t1
                break
            }
        }
        toiMaxIters = MathUtils.max(toiMaxIters, iter)
    }

    companion object {
        const val MAX_ITERATIONS: Int = 20
        const val MAX_ROOT_ITERATIONS: Int = 50
        var toiCalls: Int = 0
        var toiIters: Int = 0
        var toiMaxIters: Int = 0
        var toiRootIters: Int = 0
        var toiMaxRootIters: Int = 0
    }
}

private enum class Type {
    POINTS,
    FACE_A,
    FACE_B,
}

private class SeparationFunction {
    lateinit var m_proxyA: DistanceProxy
    lateinit var m_proxyB: DistanceProxy
    lateinit var m_type: Type
    val m_localPoint = Vec2()
    val m_axis = Vec2()
    lateinit var m_sweepA: Sweep
    lateinit var m_sweepB: Sweep

    private val localPointA = Vec2()
    private val localPointB = Vec2()
    private val pointA = Vec2()
    private val pointB = Vec2()
    private val localPointA1 = Vec2()
    private val localPointA2 = Vec2()
    private val normal = Vec2()
    private val localPointB1 = Vec2()
    private val localPointB2 = Vec2()
    private val temp = Vec2()
    private val xfa = Transform()
    private val xfb = Transform()
    private val axisA = Vec2()
    private val axisB = Vec2()

    fun initialize(cache: SimplexCache, proxyA: DistanceProxy, sweepA: Sweep, proxyB: DistanceProxy, sweepB: Sweep, t1: Float): Float {
        m_proxyA = proxyA
        m_proxyB = proxyB
        val count = cache.count
        assert(count > 0 && count < 3)
        m_sweepA = sweepA
        m_sweepB = sweepB
        m_sweepA.getTransform(xfa, t1)
        m_sweepB.getTransform(xfb, t1)

        return if (count == 1) {
            m_type = Type.POINTS
            localPointA.set(m_proxyA.getVertex(cache.indexA[0]))
            localPointB.set(m_proxyB.getVertex(cache.indexB[0]))
            Transform.mulToOutUnsafe(xfa, localPointA, pointA)
            Transform.mulToOutUnsafe(xfb, localPointB, pointB)
            m_axis.set(pointB).subLocal(pointA)
            m_axis.normalize()
        } else if (cache.indexA[0] == cache.indexA[1]) {
            m_type = Type.FACE_B
            localPointB1.set(m_proxyB.getVertex(cache.indexB[0]))
            localPointB2.set(m_proxyB.getVertex(cache.indexB[1]))
            temp.set(localPointB2).subLocal(localPointB1)
            Vec2.crossToOutUnsafe(temp, 1f, m_axis)
            m_axis.normalize()
            Rot.mulToOutUnsafe(xfb.q, m_axis, normal)
            m_localPoint.set(localPointB1).addLocal(localPointB2).mulLocal(.5f)
            Transform.mulToOutUnsafe(xfb, m_localPoint, pointB)
            localPointA.set(proxyA.getVertex(cache.indexA[0]))
            Transform.mulToOutUnsafe(xfa, localPointA, pointA)
            temp.set(pointA).subLocal(pointB)
            var s = Vec2.dot(temp, normal)
            if (s < 0f) {
                m_axis.negateLocal()
                s = -s
            }
            s
        } else {
            m_type = Type.FACE_A
            localPointA1.set(m_proxyA.getVertex(cache.indexA[0]))
            localPointA2.set(m_proxyA.getVertex(cache.indexA[1]))
            temp.set(localPointA2).subLocal(localPointA1)
            Vec2.crossToOutUnsafe(temp, 1f, m_axis)
            m_axis.normalize()
            Rot.mulToOutUnsafe(xfa.q, m_axis, normal)
            m_localPoint.set(localPointA1).addLocal(localPointA2).mulLocal(.5f)
            Transform.mulToOutUnsafe(xfa, m_localPoint, pointA)
            localPointB.set(m_proxyB.getVertex(cache.indexB[0]))
            Transform.mulToOutUnsafe(xfb, localPointB, pointB)
            temp.set(pointB).subLocal(pointA)
            var s = Vec2.dot(temp, normal)
            if (s < 0f) {
                m_axis.negateLocal()
                s = -s
            }
            s
        }
    }

    fun findMinSeparation(indexes: kotlin.IntArray, t: Float): Float {
        m_sweepA.getTransform(xfa, t)
        m_sweepB.getTransform(xfb, t)
        return when (m_type) {
            Type.POINTS -> {
                Rot.mulTransUnsafe(xfa.q, m_axis, axisA)
                Rot.mulTransUnsafe(xfb.q, m_axis.negateLocal(), axisB)
                m_axis.negateLocal()
                indexes[0] = m_proxyA.getSupport(axisA)
                indexes[1] = m_proxyB.getSupport(axisB)
                localPointA.set(m_proxyA.getVertex(indexes[0]))
                localPointB.set(m_proxyB.getVertex(indexes[1]))
                Transform.mulToOutUnsafe(xfa, localPointA, pointA)
                Transform.mulToOutUnsafe(xfb, localPointB, pointB)
                Vec2.dot(pointB.subLocal(pointA), m_axis)
            }
            Type.FACE_A -> {
                Rot.mulToOutUnsafe(xfa.q, m_axis, normal)
                Transform.mulToOutUnsafe(xfa, m_localPoint, pointA)
                Rot.mulTransUnsafe(xfb.q, normal.negateLocal(), axisB)
                normal.negateLocal()
                indexes[0] = -1
                indexes[1] = m_proxyB.getSupport(axisB)
                localPointB.set(m_proxyB.getVertex(indexes[1]))
                Transform.mulToOutUnsafe(xfb, localPointB, pointB)
                Vec2.dot(pointB.subLocal(pointA), normal)
            }
            Type.FACE_B -> {
                Rot.mulToOutUnsafe(xfb.q, m_axis, normal)
                Transform.mulToOutUnsafe(xfb, m_localPoint, pointB)
                Rot.mulTransUnsafe(xfa.q, normal.negateLocal(), axisA)
                normal.negateLocal()
                indexes[1] = -1
                indexes[0] = m_proxyA.getSupport(axisA)
                localPointA.set(m_proxyA.getVertex(indexes[0]))
                Transform.mulToOutUnsafe(xfa, localPointA, pointA)
                Vec2.dot(pointA.subLocal(pointB), normal)
            }
        }
    }

    fun evaluate(indexA: Int, indexB: Int, t: Float): Float {
        m_sweepA.getTransform(xfa, t)
        m_sweepB.getTransform(xfb, t)
        return when (m_type) {
            Type.POINTS -> {
                localPointA.set(m_proxyA.getVertex(indexA))
                localPointB.set(m_proxyB.getVertex(indexB))
                Transform.mulToOutUnsafe(xfa, localPointA, pointA)
                Transform.mulToOutUnsafe(xfb, localPointB, pointB)
                Vec2.dot(pointB.subLocal(pointA), m_axis)
            }
            Type.FACE_A -> {
                Rot.mulToOutUnsafe(xfa.q, m_axis, normal)
                Transform.mulToOutUnsafe(xfa, m_localPoint, pointA)
                localPointB.set(m_proxyB.getVertex(indexB))
                Transform.mulToOutUnsafe(xfb, localPointB, pointB)
                Vec2.dot(pointB.subLocal(pointA), normal)
            }
            Type.FACE_B -> {
                Rot.mulToOutUnsafe(xfb.q, m_axis, normal)
                Transform.mulToOutUnsafe(xfb, m_localPoint, pointB)
                localPointA.set(m_proxyA.getVertex(indexA))
                Transform.mulToOutUnsafe(xfa, localPointA, pointA)
                Vec2.dot(pointA.subLocal(pointB), normal)
            }
        }
    }
}
