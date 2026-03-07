package xyz.thewind.kbox2d.dynamics.contacts

import xyz.thewind.kbox2d.collision.Manifold
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.collision.shapes.ShapeType
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.dynamics.Fixture
import xyz.thewind.kbox2d.pooling.IWorldPool

class CircleContact(argPool: IWorldPool) : Contact(argPool) {
    fun init(fixtureA: Fixture, fixtureB: Fixture) {
        super.init(fixtureA, 0, fixtureB, 0)
        assert(m_fixtureA!!.getType() == ShapeType.CIRCLE)
        assert(m_fixtureB!!.getType() == ShapeType.CIRCLE)
    }

    override fun evaluate(manifold: Manifold, xfA: Transform, xfB: Transform) {
        pool.getCollision().collideCircles(manifold, m_fixtureA!!.getShape() as CircleShape, xfA, m_fixtureB!!.getShape() as CircleShape, xfB)
    }
}
