package xyz.thewind.kbox2d.collision.shapes

import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.RayCastInput
import xyz.thewind.kbox2d.collision.RayCastOutput
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2

class ChainShape : Shape(ShapeType.CHAIN) {
    var m_vertices: Array<Vec2>? = null
    var m_count: Int = 0
    val m_prevVertex: Vec2 = Vec2()
    val m_nextVertex: Vec2 = Vec2()
    var m_hasPrevVertex: Boolean = false
    var m_hasNextVertex: Boolean = false

    private val pool0 = EdgeShape()

    init {
        m_radius = Settings.polygonRadius
    }

    fun clear() {
        m_vertices = null
        m_count = 0
    }

    override fun getChildCount(): Int = m_count - 1

    fun getChildEdge(edge: EdgeShape, index: Int) {
        assert(0 <= index && index < m_count - 1)
        val vertices = requireNotNull(m_vertices)
        edge.m_radius = m_radius

        val v0 = vertices[index]
        val v1 = vertices[index + 1]
        edge.m_vertex1.set(v0)
        edge.m_vertex2.set(v1)

        if (index > 0) {
            edge.m_vertex0.set(vertices[index - 1])
            edge.m_hasVertex0 = true
        } else {
            edge.m_vertex0.set(m_prevVertex)
            edge.m_hasVertex0 = m_hasPrevVertex
        }

        if (index < m_count - 2) {
            edge.m_vertex3.set(vertices[index + 2])
            edge.m_hasVertex3 = true
        } else {
            edge.m_vertex3.set(m_nextVertex)
            edge.m_hasVertex3 = m_hasNextVertex
        }
    }

    override fun computeDistanceToOut(xf: Transform, p: Vec2, childIndex: Int, normalOut: Vec2): Float {
        val edge = pool0
        getChildEdge(edge, childIndex)
        return edge.computeDistanceToOut(xf, p, 0, normalOut)
    }

    override fun testPoint(xf: Transform, p: Vec2): Boolean = false

    override fun raycast(output: RayCastOutput, input: RayCastInput, transform: Transform, childIndex: Int): Boolean {
        assert(childIndex < m_count)
        val vertices = requireNotNull(m_vertices)
        val edgeShape = pool0
        val i1 = childIndex
        var i2 = childIndex + 1
        if (i2 == m_count) i2 = 0
        edgeShape.m_vertex1.set(vertices[i1])
        edgeShape.m_vertex2.set(vertices[i2])
        return edgeShape.raycast(output, input, transform, 0)
    }

    override fun computeAABB(aabb: AABB, xf: Transform, childIndex: Int) {
        assert(childIndex < m_count)
        val vertices = requireNotNull(m_vertices)
        val lower = aabb.lowerBound
        val upper = aabb.upperBound
        val i1 = childIndex
        var i2 = childIndex + 1
        if (i2 == m_count) i2 = 0

        val vi1 = vertices[i1]
        val vi2 = vertices[i2]
        val v1x = (xf.q.c * vi1.x - xf.q.s * vi1.y) + xf.p.x
        val v1y = (xf.q.s * vi1.x + xf.q.c * vi1.y) + xf.p.y
        val v2x = (xf.q.c * vi2.x - xf.q.s * vi2.y) + xf.p.x
        val v2y = (xf.q.s * vi2.x + xf.q.c * vi2.y) + xf.p.y

        lower.x = if (v1x < v2x) v1x else v2x
        lower.y = if (v1y < v2y) v1y else v2y
        upper.x = if (v1x > v2x) v1x else v2x
        upper.y = if (v1y > v2y) v1y else v2y
    }

    override fun computeMass(massData: MassData, density: Float) {
        massData.mass = 0f
        massData.center.setZero()
        massData.I = 0f
    }

    override fun clone(): Shape = ChainShape().also { clone ->
        val vertices = requireNotNull(m_vertices)
        clone.createChain(vertices, m_count)
        clone.m_prevVertex.set(m_prevVertex)
        clone.m_nextVertex.set(m_nextVertex)
        clone.m_hasPrevVertex = m_hasPrevVertex
        clone.m_hasNextVertex = m_hasNextVertex
    }

    fun createLoop(vertices: Array<Vec2>, count: Int) {
        assert(m_vertices == null && m_count == 0)
        assert(count >= 3)
        m_count = count + 1
        m_vertices = Array(m_count) { Vec2() }
        for (i in 1 until count) {
            val v1 = vertices[i - 1]
            val v2 = vertices[i]
            if (MathUtils.distanceSquared(v1, v2) < Settings.linearSlop * Settings.linearSlop) {
                throw RuntimeException("Vertices of chain shape are too close together")
            }
        }
        for (i in 0 until count) {
            m_vertices!![i] = Vec2(vertices[i])
        }
        m_vertices!![count] = Vec2(m_vertices!![0])
        m_prevVertex.set(m_vertices!![m_count - 2])
        m_nextVertex.set(m_vertices!![1])
        m_hasPrevVertex = true
        m_hasNextVertex = true
    }

    fun createChain(vertices: Array<Vec2>, count: Int) {
        assert(m_vertices == null && m_count == 0)
        assert(count >= 2)
        m_count = count
        m_vertices = Array(m_count) { Vec2() }
        for (i in 1 until m_count) {
            val v1 = vertices[i - 1]
            val v2 = vertices[i]
            if (MathUtils.distanceSquared(v1, v2) < Settings.linearSlop * Settings.linearSlop) {
                throw RuntimeException("Vertices of chain shape are too close together")
            }
        }
        for (i in 0 until m_count) {
            m_vertices!![i] = Vec2(vertices[i])
        }
        m_hasPrevVertex = false
        m_hasNextVertex = false
        m_prevVertex.setZero()
        m_nextVertex.setZero()
    }

    fun setPrevVertex(prevVertex: Vec2) {
        m_prevVertex.set(prevVertex)
        m_hasPrevVertex = true
    }

    fun setNextVertex(nextVertex: Vec2) {
        m_nextVertex.set(nextVertex)
        m_hasNextVertex = true
    }
}
