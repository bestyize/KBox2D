package xyz.thewind.kbox2d.collision.shapes

import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.RayCastInput
import xyz.thewind.kbox2d.collision.RayCastOutput
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2

class EdgeShape : Shape(ShapeType.EDGE) {
    val m_vertex1: Vec2 = Vec2()
    val m_vertex2: Vec2 = Vec2()
    val m_vertex0: Vec2 = Vec2()
    val m_vertex3: Vec2 = Vec2()
    var m_hasVertex0: Boolean = false
    var m_hasVertex3: Boolean = false

    private val normal = Vec2()

    init {
        m_radius = Settings.polygonRadius
    }

    override fun getChildCount(): Int = 1

    fun set(v1: Vec2, v2: Vec2) {
        m_vertex1.set(v1)
        m_vertex2.set(v2)
        m_hasVertex0 = false
        m_hasVertex3 = false
    }

    override fun testPoint(xf: Transform, p: Vec2): Boolean = false

    override fun computeDistanceToOut(xf: Transform, p: Vec2, childIndex: Int, normalOut: Vec2): Float {
        val xfqc = xf.q.c
        val xfqs = xf.q.s
        val xfpx = xf.p.x
        val xfpy = xf.p.y
        val v1x = (xfqc * m_vertex1.x - xfqs * m_vertex1.y) + xfpx
        val v1y = (xfqs * m_vertex1.x + xfqc * m_vertex1.y) + xfpy
        val v2x = (xfqc * m_vertex2.x - xfqs * m_vertex2.y) + xfpx
        val v2y = (xfqs * m_vertex2.x + xfqc * m_vertex2.y) + xfpy

        var dx = p.x - v1x
        var dy = p.y - v1y
        val sx = v2x - v1x
        val sy = v2y - v1y
        val ds = dx * sx + dy * sy
        if (ds > 0) {
            val s2 = sx * sx + sy * sy
            if (ds > s2) {
                dx = p.x - v2x
                dy = p.y - v2y
            } else {
                dx -= ds / s2 * sx
                dy -= ds / s2 * sy
            }
        }

        val d1 = MathUtils.sqrt(dx * dx + dy * dy)
        if (d1 > 0) {
            normalOut.x = dx / d1
            normalOut.y = dy / d1
        } else {
            normalOut.x = 0f
            normalOut.y = 0f
        }
        return d1
    }

    override fun raycast(output: RayCastOutput, input: RayCastInput, transform: Transform, childIndex: Int): Boolean {
        var tempx: Float
        var tempy: Float
        val v1 = m_vertex1
        val v2 = m_vertex2
        val xfq = transform.q
        val xfp = transform.p

        tempx = input.p1.x - xfp.x
        tempy = input.p1.y - xfp.y
        val p1x = xfq.c * tempx + xfq.s * tempy
        val p1y = -xfq.s * tempx + xfq.c * tempy

        tempx = input.p2.x - xfp.x
        tempy = input.p2.y - xfp.y
        val p2x = xfq.c * tempx + xfq.s * tempy
        val p2y = -xfq.s * tempx + xfq.c * tempy

        val dx = p2x - p1x
        val dy = p2y - p1y

        normal.x = v2.y - v1.y
        normal.y = v1.x - v2.x
        normal.normalize()
        val normalx = normal.x
        val normaly = normal.y

        tempx = v1.x - p1x
        tempy = v1.y - p1y
        val numerator = normalx * tempx + normaly * tempy
        val denominator = normalx * dx + normaly * dy
        if (denominator == 0.0f) return false

        val t = numerator / denominator
        if (t < 0.0f || 1.0f < t) return false

        val qx = p1x + t * dx
        val qy = p1y + t * dy
        val rx = v2.x - v1.x
        val ry = v2.y - v1.y
        val rr = rx * rx + ry * ry
        if (rr == 0.0f) return false
        tempx = qx - v1.x
        tempy = qy - v1.y
        val s = (tempx * rx + tempy * ry) / rr
        if (s < 0.0f || 1.0f < s) return false

        output.fraction = t
        if (numerator > 0.0f) {
            output.normal.x = -xfq.c * normal.x + xfq.s * normal.y
            output.normal.y = -xfq.s * normal.x - xfq.c * normal.y
        } else {
            output.normal.x = xfq.c * normal.x - xfq.s * normal.y
            output.normal.y = xfq.s * normal.x + xfq.c * normal.y
        }
        return true
    }

    override fun computeAABB(aabb: AABB, xf: Transform, childIndex: Int) {
        val lowerBound = aabb.lowerBound
        val upperBound = aabb.upperBound
        val xfq = xf.q
        val v1x = (xfq.c * m_vertex1.x - xfq.s * m_vertex1.y) + xf.p.x
        val v1y = (xfq.s * m_vertex1.x + xfq.c * m_vertex1.y) + xf.p.y
        val v2x = (xfq.c * m_vertex2.x - xfq.s * m_vertex2.y) + xf.p.x
        val v2y = (xfq.s * m_vertex2.x + xfq.c * m_vertex2.y) + xf.p.y

        lowerBound.x = if (v1x < v2x) v1x else v2x
        lowerBound.y = if (v1y < v2y) v1y else v2y
        upperBound.x = if (v1x > v2x) v1x else v2x
        upperBound.y = if (v1y > v2y) v1y else v2y

        lowerBound.x -= m_radius
        lowerBound.y -= m_radius
        upperBound.x += m_radius
        upperBound.y += m_radius
    }

    override fun computeMass(massData: MassData, density: Float) {
        massData.mass = 0.0f
        massData.center.set(m_vertex1).addLocal(m_vertex2).mulLocal(0.5f)
        massData.I = 0.0f
    }

    override fun clone(): Shape = EdgeShape().also { edge ->
        edge.m_radius = m_radius
        edge.m_hasVertex0 = m_hasVertex0
        edge.m_hasVertex3 = m_hasVertex3
        edge.m_vertex0.set(m_vertex0)
        edge.m_vertex1.set(m_vertex1)
        edge.m_vertex2.set(m_vertex2)
        edge.m_vertex3.set(m_vertex3)
    }
}
