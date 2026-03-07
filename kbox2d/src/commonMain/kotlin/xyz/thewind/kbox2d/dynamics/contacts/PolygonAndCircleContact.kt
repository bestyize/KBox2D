package xyz.thewind.kbox2d.dynamics.contacts

import xyz.thewind.kbox2d.collision.Manifold
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.collision.shapes.PolygonShape
import xyz.thewind.kbox2d.collision.shapes.ShapeType
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.dynamics.Fixture
import xyz.thewind.kbox2d.pooling.IWorldPool

class PolygonAndCircleContact(argPool: IWorldPool) : Contact(argPool) {
    fun init(fixtureA: Fixture, fixtureB: Fixture) {
        super.init(fixtureA, 0, fixtureB, 0)
        assert(m_fixtureA!!.getType() == ShapeType.POLYGON)
        assert(m_fixtureB!!.getType() == ShapeType.CIRCLE)
    }

    override fun evaluate(manifold: Manifold, xfA: Transform, xfB: Transform) {
        pool.getCollision().collidePolygonAndCircle(manifold, m_fixtureA!!.getShape() as PolygonShape, xfA, m_fixtureB!!.getShape() as CircleShape, xfB)
    }
}
