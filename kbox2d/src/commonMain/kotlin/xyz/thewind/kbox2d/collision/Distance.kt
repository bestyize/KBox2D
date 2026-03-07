package xyz.thewind.kbox2d.collision

import xyz.thewind.kbox2d.collision.shapes.ChainShape
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.collision.shapes.EdgeShape
import xyz.thewind.kbox2d.collision.shapes.PolygonShape
import xyz.thewind.kbox2d.collision.shapes.Shape
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2

class Distance {
    private class SimplexVertex {
        val wA = Vec2()
        val wB = Vec2()
        val w = Vec2()
        var a: Float = 0f
        var indexA: Int = 0
        var indexB: Int = 0

        fun set(sv: SimplexVertex) {
            wA.set(sv.wA)
            wB.set(sv.wB)
            w.set(sv.w)
            a = sv.a
            indexA = sv.indexA
            indexB = sv.indexB
        }
    }

    class SimplexCache {
        var metric: Float = 0f
        var count: Int = 0
        val indexA: kotlin.IntArray = kotlin.IntArray(3) { Int.MAX_VALUE }
        val indexB: kotlin.IntArray = kotlin.IntArray(3) { Int.MAX_VALUE }

        fun set(sc: SimplexCache) {
            sc.indexA.copyInto(indexA)
            sc.indexB.copyInto(indexB)
            metric = sc.metric
            count = sc.count
        }
    }

    private class Simplex {
        val m_v1 = SimplexVertex()
        val m_v2 = SimplexVertex()
        val m_v3 = SimplexVertex()
        val vertices = arrayOf(m_v1, m_v2, m_v3)
        var m_count: Int = 0

        private val e12 = Vec2()
        private val case2 = Vec2()
        private val case22 = Vec2()
        private val case3 = Vec2()
        private val case33 = Vec2()
        private val e13 = Vec2()
        private val e23 = Vec2()
        private val w1 = Vec2()
        private val w2 = Vec2()
        private val w3 = Vec2()

        fun readCache(cache: SimplexCache, proxyA: DistanceProxy, transformA: Transform, proxyB: DistanceProxy, transformB: Transform) {
            assert(cache.count <= 3)
            m_count = cache.count
            for (i in 0 until m_count) {
                val v = vertices[i]
                v.indexA = cache.indexA[i]
                v.indexB = cache.indexB[i]
                val wALocal = proxyA.getVertex(v.indexA)
                val wBLocal = proxyB.getVertex(v.indexB)
                Transform.mulToOutUnsafe(transformA, wALocal, v.wA)
                Transform.mulToOutUnsafe(transformB, wBLocal, v.wB)
                v.w.set(v.wB).subLocal(v.wA)
                v.a = 0f
            }

            if (m_count > 1) {
                val metric1 = cache.metric
                val metric2 = getMetric()
                if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < Settings.EPSILON) {
                    m_count = 0
                }
            }

            if (m_count == 0) {
                val v = vertices[0]
                v.indexA = 0
                v.indexB = 0
                val wALocal = proxyA.getVertex(0)
                val wBLocal = proxyB.getVertex(0)
                Transform.mulToOutUnsafe(transformA, wALocal, v.wA)
                Transform.mulToOutUnsafe(transformB, wBLocal, v.wB)
                v.w.set(v.wB).subLocal(v.wA)
                m_count = 1
            }
        }

        fun writeCache(cache: SimplexCache) {
            cache.metric = getMetric()
            cache.count = m_count
            for (i in 0 until m_count) {
                cache.indexA[i] = vertices[i].indexA
                cache.indexB[i] = vertices[i].indexB
            }
        }

        fun getSearchDirection(out: Vec2) {
            when (m_count) {
                1 -> out.set(m_v1.w).negateLocal()
                2 -> {
                    e12.set(m_v2.w).subLocal(m_v1.w)
                    out.set(m_v1.w).negateLocal()
                    val sgn = Vec2.cross(e12, out)
                    if (sgn > 0f) {
                        Vec2.crossToOutUnsafe(1f, e12, out)
                    } else {
                        Vec2.crossToOutUnsafe(e12, 1f, out)
                    }
                }
                else -> {
                    assert(false)
                    out.setZero()
                }
            }
        }

        fun getClosestPoint(out: Vec2) {
            when (m_count) {
                0 -> {
                    assert(false)
                    out.setZero()
                }
                1 -> out.set(m_v1.w)
                2 -> {
                    case22.set(m_v2.w).mulLocal(m_v2.a)
                    case2.set(m_v1.w).mulLocal(m_v1.a).addLocal(case22)
                    out.set(case2)
                }
                3 -> out.setZero()
                else -> {
                    assert(false)
                    out.setZero()
                }
            }
        }

        fun getWitnessPoints(pA: Vec2, pB: Vec2) {
            when (m_count) {
                0 -> assert(false)
                1 -> {
                    pA.set(m_v1.wA)
                    pB.set(m_v1.wB)
                }
                2 -> {
                    case2.set(m_v1.wA).mulLocal(m_v1.a)
                    pA.set(m_v2.wA).mulLocal(m_v2.a).addLocal(case2)
                    case2.set(m_v1.wB).mulLocal(m_v1.a)
                    pB.set(m_v2.wB).mulLocal(m_v2.a).addLocal(case2)
                }
                3 -> {
                    pA.set(m_v1.wA).mulLocal(m_v1.a)
                    case3.set(m_v2.wA).mulLocal(m_v2.a)
                    case33.set(m_v3.wA).mulLocal(m_v3.a)
                    pA.addLocal(case3).addLocal(case33)
                    pB.set(pA)
                }
                else -> assert(false)
            }
        }

        fun getMetric(): Float = when (m_count) {
            0 -> {
                assert(false)
                0f
            }
            1 -> 0f
            2 -> MathUtils.distance(m_v1.w, m_v2.w)
            3 -> {
                case3.set(m_v2.w).subLocal(m_v1.w)
                case33.set(m_v3.w).subLocal(m_v1.w)
                Vec2.cross(case3, case33)
            }
            else -> {
                assert(false)
                0f
            }
        }

        fun solve2() {
            val w1 = m_v1.w
            val w2 = m_v2.w
            e12.set(w2).subLocal(w1)
            val d12_2 = -Vec2.dot(w1, e12)
            if (d12_2 <= 0f) {
                m_v1.a = 1f
                m_count = 1
                return
            }
            val d12_1 = Vec2.dot(w2, e12)
            if (d12_1 <= 0f) {
                m_v2.a = 1f
                m_count = 1
                m_v1.set(m_v2)
                return
            }
            val invD12 = 1f / (d12_1 + d12_2)
            m_v1.a = d12_1 * invD12
            m_v2.a = d12_2 * invD12
            m_count = 2
        }

        fun solve3() {
            w1.set(m_v1.w)
            w2.set(m_v2.w)
            w3.set(m_v3.w)

            e12.set(w2).subLocal(w1)
            val w1e12 = Vec2.dot(w1, e12)
            val w2e12 = Vec2.dot(w2, e12)
            val d12_1 = w2e12
            val d12_2 = -w1e12

            e13.set(w3).subLocal(w1)
            val w1e13 = Vec2.dot(w1, e13)
            val w3e13 = Vec2.dot(w3, e13)
            val d13_1 = w3e13
            val d13_2 = -w1e13

            e23.set(w3).subLocal(w2)
            val w2e23 = Vec2.dot(w2, e23)
            val w3e23 = Vec2.dot(w3, e23)
            val d23_1 = w3e23
            val d23_2 = -w2e23

            val n123 = Vec2.cross(e12, e13)
            val d123_1 = n123 * Vec2.cross(w2, w3)
            val d123_2 = n123 * Vec2.cross(w3, w1)
            val d123_3 = n123 * Vec2.cross(w1, w2)

            if (d12_2 <= 0f && d13_2 <= 0f) {
                m_v1.a = 1f
                m_count = 1
                return
            }
            if (d12_1 > 0f && d12_2 > 0f && d123_3 <= 0f) {
                val invD12 = 1f / (d12_1 + d12_2)
                m_v1.a = d12_1 * invD12
                m_v2.a = d12_2 * invD12
                m_count = 2
                return
            }
            if (d13_1 > 0f && d13_2 > 0f && d123_2 <= 0f) {
                val invD13 = 1f / (d13_1 + d13_2)
                m_v1.a = d13_1 * invD13
                m_v3.a = d13_2 * invD13
                m_count = 2
                m_v2.set(m_v3)
                return
            }
            if (d12_1 <= 0f && d23_2 <= 0f) {
                m_v2.a = 1f
                m_count = 1
                m_v1.set(m_v2)
                return
            }
            if (d13_1 <= 0f && d23_1 <= 0f) {
                m_v3.a = 1f
                m_count = 1
                m_v1.set(m_v3)
                return
            }
            if (d23_1 > 0f && d23_2 > 0f && d123_1 <= 0f) {
                val invD23 = 1f / (d23_1 + d23_2)
                m_v2.a = d23_1 * invD23
                m_v3.a = d23_2 * invD23
                m_count = 2
                m_v1.set(m_v3)
                return
            }
            val invD123 = 1f / (d123_1 + d123_2 + d123_3)
            m_v1.a = d123_1 * invD123
            m_v2.a = d123_2 * invD123
            m_v3.a = d123_3 * invD123
            m_count = 3
        }
    }

    class DistanceProxy {
        val m_vertices: Array<Vec2> = Array(Settings.maxPolygonVertices) { Vec2() }
        var m_count: Int = 0
        var m_radius: Float = 0f
        private val m_buffer: Array<Vec2> = Array(2) { Vec2() }

        fun set(shape: Shape, index: Int) {
            when (shape.getType()) {
                xyz.thewind.kbox2d.collision.shapes.ShapeType.CIRCLE -> {
                    val circle = shape as CircleShape
                    m_vertices[0].set(circle.m_p)
                    m_count = 1
                    m_radius = circle.m_radius
                }
                xyz.thewind.kbox2d.collision.shapes.ShapeType.POLYGON -> {
                    val poly = shape as PolygonShape
                    m_count = poly.m_count
                    m_radius = poly.m_radius
                    for (i in 0 until m_count) {
                        m_vertices[i].set(poly.m_vertices[i])
                    }
                }
                xyz.thewind.kbox2d.collision.shapes.ShapeType.CHAIN -> {
                    val chain = shape as ChainShape
                    assert(0 <= index && index < chain.m_count)
                    val vertices = requireNotNull(chain.m_vertices)
                    m_buffer[0].set(vertices[index])
                    if (index + 1 < chain.m_count) {
                        m_buffer[1].set(vertices[index + 1])
                    } else {
                        m_buffer[1].set(vertices[0])
                    }
                    m_vertices[0].set(m_buffer[0])
                    m_vertices[1].set(m_buffer[1])
                    m_count = 2
                    m_radius = chain.m_radius
                }
                xyz.thewind.kbox2d.collision.shapes.ShapeType.EDGE -> {
                    val edge = shape as EdgeShape
                    m_vertices[0].set(edge.m_vertex1)
                    m_vertices[1].set(edge.m_vertex2)
                    m_count = 2
                    m_radius = edge.m_radius
                }
            }
        }

        fun getSupport(d: Vec2): Int {
            var bestIndex = 0
            var bestValue = Vec2.dot(m_vertices[0], d)
            for (i in 1 until m_count) {
                val value = Vec2.dot(m_vertices[i], d)
                if (value > bestValue) {
                    bestIndex = i
                    bestValue = value
                }
            }
            return bestIndex
        }

        fun getSupportVertex(d: Vec2): Vec2 = m_vertices[getSupport(d)]

        fun getVertexCount(): Int = m_count

        fun getVertex(index: Int): Vec2 {
            assert(index in 0 until m_count)
            return m_vertices[index]
        }
    }

    private val simplex = Simplex()
    private val saveA = kotlin.IntArray(3)
    private val saveB = kotlin.IntArray(3)
    private val closestPoint = Vec2()
    private val d = Vec2()
    private val temp = Vec2()
    private val normal = Vec2()

    fun distance(output: DistanceOutput, cache: SimplexCache, input: DistanceInput) {
        GJK_CALLS++
        val proxyA = input.proxyA
        val proxyB = input.proxyB
        val transformA = input.transformA
        val transformB = input.transformB

        simplex.readCache(cache, proxyA, transformA, proxyB, transformB)
        val vertices = simplex.vertices
        simplex.getClosestPoint(closestPoint)
        var distanceSqr1 = closestPoint.lengthSquared()
        var distanceSqr2 = distanceSqr1
        var iter = 0
        while (iter < MAX_ITERS) {
            val saveCount = simplex.m_count
            for (i in 0 until saveCount) {
                saveA[i] = vertices[i].indexA
                saveB[i] = vertices[i].indexB
            }

            when (simplex.m_count) {
                1 -> Unit
                2 -> simplex.solve2()
                3 -> simplex.solve3()
                else -> assert(false)
            }

            if (simplex.m_count == 3) break

            simplex.getClosestPoint(closestPoint)
            distanceSqr2 = closestPoint.lengthSquared()
            distanceSqr1 = distanceSqr2
            simplex.getSearchDirection(d)
            if (d.lengthSquared() < Settings.EPSILON * Settings.EPSILON) {
                break
            }

            val vertex = vertices[simplex.m_count]
            Rot.mulTransUnsafe(transformA.q, d.negateLocal(), temp)
            vertex.indexA = proxyA.getSupport(temp)
            Transform.mulToOutUnsafe(transformA, proxyA.getVertex(vertex.indexA), vertex.wA)
            Rot.mulTransUnsafe(transformB.q, d.negateLocal(), temp)
            vertex.indexB = proxyB.getSupport(temp)
            Transform.mulToOutUnsafe(transformB, proxyB.getVertex(vertex.indexB), vertex.wB)
            vertex.w.set(vertex.wB).subLocal(vertex.wA)

            ++iter
            ++GJK_ITERS

            var duplicate = false
            for (i in 0 until saveCount) {
                if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i]) {
                    duplicate = true
                    break
                }
            }
            if (duplicate) break
            ++simplex.m_count
        }

        GJK_MAX_ITERS = MathUtils.max(GJK_MAX_ITERS, iter)
        simplex.getWitnessPoints(output.pointA, output.pointB)
        output.distance = MathUtils.distance(output.pointA, output.pointB)
        output.iterations = iter
        simplex.writeCache(cache)

        if (input.useRadii) {
            val rA = proxyA.m_radius
            val rB = proxyB.m_radius
            if (output.distance > rA + rB && output.distance > Settings.EPSILON) {
                output.distance -= rA + rB
                normal.set(output.pointB).subLocal(output.pointA)
                normal.normalize()
                temp.set(normal).mulLocal(rA)
                output.pointA.addLocal(temp)
                temp.set(normal).mulLocal(rB)
                output.pointB.subLocal(temp)
            } else {
                output.pointA.addLocal(output.pointB).mulLocal(.5f)
                output.pointB.set(output.pointA)
                output.distance = 0f
            }
        }
    }

    companion object {
        const val MAX_ITERS: Int = 20
        var GJK_CALLS: Int = 0
        var GJK_ITERS: Int = 0
        var GJK_MAX_ITERS: Int = 20
    }
}
