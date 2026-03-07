package xyz.thewind.kbox2d.collision.shapes

import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.RayCastInput
import xyz.thewind.kbox2d.collision.RayCastOutput
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2

abstract class Shape(val m_type: ShapeType) {
    var m_radius: Float = 0f

    fun getType(): ShapeType = m_type

    fun getRadius(): Float = m_radius

    fun setRadius(radius: Float) {
        m_radius = radius
    }

    abstract fun getChildCount(): Int
    abstract fun testPoint(xf: Transform, p: Vec2): Boolean
    abstract fun raycast(output: RayCastOutput, input: RayCastInput, transform: Transform, childIndex: Int): Boolean
    abstract fun computeAABB(aabb: AABB, xf: Transform, childIndex: Int)
    abstract fun computeMass(massData: MassData, density: Float)
    abstract fun computeDistanceToOut(xf: Transform, p: Vec2, childIndex: Int, normalOut: Vec2): Float
    abstract fun clone(): Shape
}
