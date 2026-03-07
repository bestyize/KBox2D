package xyz.thewind.kbox2d.collision.shapes

import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.RayCastInput
import xyz.thewind.kbox2d.collision.RayCastOutput
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.pooling.arrays.IntArray
import xyz.thewind.kbox2d.pooling.arrays.Vec2Array

class PolygonShape : Shape(ShapeType.POLYGON) {
    val m_centroid: Vec2 = Vec2()
    val m_vertices: Array<Vec2> = Array(Settings.maxPolygonVertices) { Vec2() }
    val m_normals: Array<Vec2> = Array(Settings.maxPolygonVertices) { Vec2() }
    var m_count: Int = 0

    private val pool1 = Vec2()
    private val pool2 = Vec2()
    private val pool3 = Vec2()
    private val pool4 = Vec2()
    private val poolt1 = Transform()

    init {
        setRadius(Settings.polygonRadius)
        m_centroid.setZero()
    }

    override fun clone(): Shape = PolygonShape().also { shape ->
        shape.m_centroid.set(m_centroid)
        for (i in shape.m_normals.indices) {
            shape.m_normals[i].set(m_normals[i])
            shape.m_vertices[i].set(m_vertices[i])
        }
        shape.setRadius(getRadius())
        shape.m_count = m_count
    }

    fun set(vertices: Array<Vec2>, count: Int) {
        set(vertices, count, null, null)
    }

    fun set(verts: Array<Vec2>, num: Int, vecPool: Vec2Array?, intPool: IntArray?) {
        assert(3 <= num && num <= Settings.maxPolygonVertices)
        if (num < 3) {
            setAsBox(1f, 1f)
            return
        }

        var n = MathUtils.min(num, Settings.maxPolygonVertices)
        val ps = vecPool?.get(Settings.maxPolygonVertices) ?: Array(Settings.maxPolygonVertices) { Vec2() }
        var tempCount = 0
        for (i in 0 until n) {
            val v = verts[i]
            var unique = true
            for (j in 0 until tempCount) {
                if (MathUtils.distanceSquared(v, ps[j]) < 0.5f * Settings.linearSlop) {
                    unique = false
                    break
                }
            }
            if (unique) {
                ps[tempCount].set(v)
                tempCount++
            }
        }

        n = tempCount
        if (n < 3) {
            setAsBox(1f, 1f)
            return
        }

        val hull = intPool?.get(Settings.maxPolygonVertices) ?: kotlin.IntArray(Settings.maxPolygonVertices)
        var i0 = 0
        var x0 = ps[0].x
        for (i in 1 until n) {
            val x = ps[i].x
            if (x > x0 || (x == x0 && ps[i].y < ps[i0].y)) {
                i0 = i
                x0 = x
            }
        }

        var m = 0
        var ih = i0
        while (true) {
            hull[m] = ih
            var ie = 0
            for (j in 1 until n) {
                if (ie == ih) {
                    ie = j
                    continue
                }
                val r = pool1.set(ps[ie]).subLocal(ps[hull[m]])
                val v = pool2.set(ps[j]).subLocal(ps[hull[m]])
                val c = Vec2.cross(r, v)
                if (c < 0.0f) {
                    ie = j
                }
                if (c == 0.0f && v.lengthSquared() > r.lengthSquared()) {
                    ie = j
                }
            }
            ++m
            ih = ie
            if (ie == i0) break
        }

        m_count = m
        for (i in 0 until m_count) {
            m_vertices[i].set(ps[hull[i]])
        }

        val edge = pool1
        for (i in 0 until m_count) {
            val i1 = i
            val i2 = if (i + 1 < m_count) i + 1 else 0
            edge.set(m_vertices[i2]).subLocal(m_vertices[i1])
            assert(edge.lengthSquared() > Settings.EPSILON * Settings.EPSILON)
            Vec2.crossToOutUnsafe(edge, 1f, m_normals[i])
            m_normals[i].normalize()
        }

        computeCentroidToOut(m_vertices, m_count, m_centroid)
    }

    fun setAsBox(hx: Float, hy: Float) {
        m_count = 4
        m_vertices[0].set(-hx, -hy)
        m_vertices[1].set(hx, -hy)
        m_vertices[2].set(hx, hy)
        m_vertices[3].set(-hx, hy)
        m_normals[0].set(0f, -1f)
        m_normals[1].set(1f, 0f)
        m_normals[2].set(0f, 1f)
        m_normals[3].set(-1f, 0f)
        m_centroid.setZero()
    }

    fun setAsBox(hx: Float, hy: Float, center: Vec2, angle: Float) {
        m_count = 4
        m_vertices[0].set(-hx, -hy)
        m_vertices[1].set(hx, -hy)
        m_vertices[2].set(hx, hy)
        m_vertices[3].set(-hx, hy)
        m_normals[0].set(0f, -1f)
        m_normals[1].set(1f, 0f)
        m_normals[2].set(0f, 1f)
        m_normals[3].set(-1f, 0f)
        m_centroid.set(center)

        val xf = poolt1
        xf.p.set(center)
        xf.q.set(angle)
        for (i in 0 until m_count) {
            Transform.mulToOut(xf, m_vertices[i], m_vertices[i])
            Rot.mulToOut(xf.q, m_normals[i], m_normals[i])
        }
    }

    override fun getChildCount(): Int = 1

    override fun testPoint(xf: Transform, p: Vec2): Boolean {
        var tempx = p.x - xf.p.x
        var tempy = p.y - xf.p.y
        val pLocalx = xf.q.c * tempx + xf.q.s * tempy
        val pLocaly = -xf.q.s * tempx + xf.q.c * tempy
        for (i in 0 until m_count) {
            val vertex = m_vertices[i]
            val normal = m_normals[i]
            tempx = pLocalx - vertex.x
            tempy = pLocaly - vertex.y
            val dot = normal.x * tempx + normal.y * tempy
            if (dot > 0.0f) return false
        }
        return true
    }

    override fun computeAABB(aabb: AABB, xf: Transform, childIndex: Int) {
        val lower = aabb.lowerBound
        val upper = aabb.upperBound
        val v1 = m_vertices[0]
        val xfqc = xf.q.c
        val xfqs = xf.q.s
        val xfpx = xf.p.x
        val xfpy = xf.p.y
        lower.x = (xfqc * v1.x - xfqs * v1.y) + xfpx
        lower.y = (xfqs * v1.x + xfqc * v1.y) + xfpy
        upper.x = lower.x
        upper.y = lower.y

        for (i in 1 until m_count) {
            val v2 = m_vertices[i]
            val vx = (xfqc * v2.x - xfqs * v2.y) + xfpx
            val vy = (xfqs * v2.x + xfqc * v2.y) + xfpy
            lower.x = if (lower.x < vx) lower.x else vx
            lower.y = if (lower.y < vy) lower.y else vy
            upper.x = if (upper.x > vx) upper.x else vx
            upper.y = if (upper.y > vy) upper.y else vy
        }

        lower.x -= m_radius
        lower.y -= m_radius
        upper.x += m_radius
        upper.y += m_radius
    }

    fun getVertexCount(): Int = m_count

    fun getVertex(index: Int): Vec2 {
        assert(index in 0 until m_count)
        return m_vertices[index]
    }

    override fun computeDistanceToOut(xf: Transform, p: Vec2, childIndex: Int, normalOut: Vec2): Float {
        val xfqc = xf.q.c
        val xfqs = xf.q.s
        var tx = p.x - xf.p.x
        var ty = p.y - xf.p.y
        val pLocalx = xfqc * tx + xfqs * ty
        val pLocaly = -xfqs * tx + xfqc * ty

        var maxDistance = -Float.MAX_VALUE
        var normalForMaxDistanceX = pLocalx
        var normalForMaxDistanceY = pLocaly

        for (i in 0 until m_count) {
            val vertex = m_vertices[i]
            val normal = m_normals[i]
            tx = pLocalx - vertex.x
            ty = pLocaly - vertex.y
            val dot = normal.x * tx + normal.y * ty
            if (dot > maxDistance) {
                maxDistance = dot
                normalForMaxDistanceX = normal.x
                normalForMaxDistanceY = normal.y
            }
        }

        return if (maxDistance > 0) {
            var minDistanceX = normalForMaxDistanceX
            var minDistanceY = normalForMaxDistanceY
            var minDistance2 = maxDistance * maxDistance
            for (i in 0 until m_count) {
                val vertex = m_vertices[i]
                val distanceVecX = pLocalx - vertex.x
                val distanceVecY = pLocaly - vertex.y
                val distance2 = distanceVecX * distanceVecX + distanceVecY * distanceVecY
                if (minDistance2 > distance2) {
                    minDistanceX = distanceVecX
                    minDistanceY = distanceVecY
                    minDistance2 = distance2
                }
            }
            val distance = MathUtils.sqrt(minDistance2)
            normalOut.x = xfqc * minDistanceX - xfqs * minDistanceY
            normalOut.y = xfqs * minDistanceX + xfqc * minDistanceY
            normalOut.normalize()
            distance
        } else {
            val distance = maxDistance
            normalOut.x = xfqc * normalForMaxDistanceX - xfqs * normalForMaxDistanceY
            normalOut.y = xfqs * normalForMaxDistanceX + xfqc * normalForMaxDistanceY
            distance
        }
    }

    override fun raycast(output: RayCastOutput, input: RayCastInput, transform: Transform, childIndex: Int): Boolean {
        val xfqc = transform.q.c
        val xfqs = transform.q.s
        val xfp = transform.p
        var tempx = input.p1.x - xfp.x
        var tempy = input.p1.y - xfp.y
        val p1x = xfqc * tempx + xfqs * tempy
        val p1y = -xfqs * tempx + xfqc * tempy

        tempx = input.p2.x - xfp.x
        tempy = input.p2.y - xfp.y
        val p2x = xfqc * tempx + xfqs * tempy
        val p2y = -xfqs * tempx + xfqc * tempy

        val dx = p2x - p1x
        val dy = p2y - p1y
        var lower = 0f
        var upper = input.maxFraction
        var index = -1

        for (i in 0 until m_count) {
            val normal = m_normals[i]
            val vertex = m_vertices[i]
            val tempxn = vertex.x - p1x
            val tempyn = vertex.y - p1y
            val numerator = normal.x * tempxn + normal.y * tempyn
            val denominator = normal.x * dx + normal.y * dy

            if (denominator == 0.0f) {
                if (numerator < 0.0f) return false
            } else {
                if (denominator < 0.0f && numerator < lower * denominator) {
                    lower = numerator / denominator
                    index = i
                } else if (denominator > 0.0f && numerator < upper * denominator) {
                    upper = numerator / denominator
                }
            }
            if (upper < lower) return false
        }

        if (index >= 0) {
            output.fraction = lower
            val normal = m_normals[index]
            output.normal.x = xfqc * normal.x - xfqs * normal.y
            output.normal.y = xfqs * normal.x + xfqc * normal.y
            return true
        }
        return false
    }

    fun computeCentroidToOut(vs: Array<Vec2>, count: Int, out: Vec2) {
        assert(count >= 3)
        out.set(0f, 0f)
        var area = 0f
        val pRef = pool1
        pRef.setZero()
        val e1 = pool2
        val e2 = pool3
        val inv3 = 1.0f / 3.0f

        for (i in 0 until count) {
            val p1 = pRef
            val p2 = vs[i]
            val p3 = if (i + 1 < count) vs[i + 1] else vs[0]
            e1.set(p2).subLocal(p1)
            e2.set(p3).subLocal(p1)
            val D = Vec2.cross(e1, e2)
            val triangleArea = 0.5f * D
            area += triangleArea
            e1.set(p1).addLocal(p2).addLocal(p3).mulLocal(triangleArea * inv3)
            out.addLocal(e1)
        }

        assert(area > Settings.EPSILON)
        out.mulLocal(1.0f / area)
    }

    override fun computeMass(massData: MassData, density: Float) {
        assert(m_count >= 3)
        val center = pool1
        center.setZero()
        var area = 0f
        var I = 0f
        val s = pool2
        s.setZero()
        for (i in 0 until m_count) {
            s.addLocal(m_vertices[i])
        }
        s.mulLocal(1.0f / m_count)

        val kInv3 = 1.0f / 3.0f
        val e1 = pool3
        val e2 = pool4

        for (i in 0 until m_count) {
            e1.set(m_vertices[i]).subLocal(s)
            e2.set(s).negateLocal().addLocal(if (i + 1 < m_count) m_vertices[i + 1] else m_vertices[0])
            val D = Vec2.cross(e1, e2)
            val triangleArea = 0.5f * D
            area += triangleArea
            center.x += triangleArea * kInv3 * (e1.x + e2.x)
            center.y += triangleArea * kInv3 * (e1.y + e2.y)

            val ex1 = e1.x
            val ey1 = e1.y
            val ex2 = e2.x
            val ey2 = e2.y
            val intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2
            val inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2
            I += (0.25f * kInv3 * D) * (intx2 + inty2)
        }

        massData.mass = density * area
        assert(area > Settings.EPSILON)
        center.mulLocal(1.0f / area)
        massData.center.set(center).addLocal(s)
        massData.I = I * density
        massData.I += massData.mass * Vec2.dot(massData.center, massData.center)
    }

    fun validate(): Boolean {
        for (i in 0 until m_count) {
            val i1 = i
            val i2 = if (i < m_count - 1) i1 + 1 else 0
            val p = m_vertices[i1]
            val e = pool1.set(m_vertices[i2]).subLocal(p)
            for (j in 0 until m_count) {
                if (j == i1 || j == i2) continue
                val v = pool2.set(m_vertices[j]).subLocal(p)
                val c = Vec2.cross(e, v)
                if (c < 0.0f) return false
            }
        }
        return true
    }

    fun getVertices(): Array<Vec2> = m_vertices

    fun getNormals(): Array<Vec2> = m_normals

    fun centroid(xf: Transform): Vec2 = Transform.mul(xf, m_centroid)

    fun centroidToOut(xf: Transform, out: Vec2): Vec2 {
        Transform.mulToOutUnsafe(xf, m_centroid, out)
        return out
    }
}
