package xyz.thewind.kbox2d.collision.shapes

import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.RayCastInput
import xyz.thewind.kbox2d.collision.RayCastOutput
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2

class CircleShape : Shape(ShapeType.CIRCLE) {
    val m_p: Vec2 = Vec2()

    init {
        m_radius = 0f
    }

    override fun clone(): Shape = CircleShape().also {
        it.m_p.x = m_p.x
        it.m_p.y = m_p.y
        it.m_radius = m_radius
    }

    override fun getChildCount(): Int = 1

    fun getSupport(d: Vec2): Int = 0

    fun getSupportVertex(d: Vec2): Vec2 = m_p

    fun getVertexCount(): Int = 1

    fun getVertex(index: Int): Vec2 {
        assert(index == 0)
        return m_p
    }

    override fun testPoint(xf: Transform, p: Vec2): Boolean {
        val q = xf.q
        val tp = xf.p
        val centerx = -(q.c * m_p.x - q.s * m_p.y + tp.x - p.x)
        val centery = -(q.s * m_p.x + q.c * m_p.y + tp.y - p.y)
        return centerx * centerx + centery * centery <= m_radius * m_radius
    }

    override fun computeDistanceToOut(xf: Transform, p: Vec2, childIndex: Int, normalOut: Vec2): Float {
        val xfq = xf.q
        val centerx = xfq.c * m_p.x - xfq.s * m_p.y + xf.p.x
        val centery = xfq.s * m_p.x + xfq.c * m_p.y + xf.p.y
        val dx = p.x - centerx
        val dy = p.y - centery
        val d1 = MathUtils.sqrt(dx * dx + dy * dy)
        normalOut.x = dx / d1
        normalOut.y = dy / d1
        return d1 - m_radius
    }

    override fun raycast(output: RayCastOutput, input: RayCastInput, transform: Transform, childIndex: Int): Boolean {
        val positionx = transform.q.c * m_p.x - transform.q.s * m_p.y + transform.p.x
        val positiony = transform.q.s * m_p.x + transform.q.c * m_p.y + transform.p.y
        val sx = input.p1.x - positionx
        val sy = input.p1.y - positiony
        val b = sx * sx + sy * sy - m_radius * m_radius
        val rx = input.p2.x - input.p1.x
        val ry = input.p2.y - input.p1.y
        val c = sx * rx + sy * ry
        val rr = rx * rx + ry * ry
        val sigma = c * c - rr * b
        if (sigma < 0.0f || rr < Settings.EPSILON) return false
        var a = -(c + MathUtils.sqrt(sigma))
        if (0.0f <= a && a <= input.maxFraction * rr) {
            a /= rr
            output.fraction = a
            output.normal.x = rx * a + sx
            output.normal.y = ry * a + sy
            output.normal.normalize()
            return true
        }
        return false
    }

    override fun computeAABB(aabb: AABB, xf: Transform, childIndex: Int) {
        val px = xf.q.c * m_p.x - xf.q.s * m_p.y + xf.p.x
        val py = xf.q.s * m_p.x + xf.q.c * m_p.y + xf.p.y
        aabb.lowerBound.x = px - m_radius
        aabb.lowerBound.y = py - m_radius
        aabb.upperBound.x = px + m_radius
        aabb.upperBound.y = py + m_radius
    }

    override fun computeMass(massData: MassData, density: Float) {
        massData.mass = density * Settings.PI * m_radius * m_radius
        massData.center.x = m_p.x
        massData.center.y = m_p.y
        massData.I = massData.mass * (0.5f * m_radius * m_radius + (m_p.x * m_p.x + m_p.y * m_p.y))
    }
}
