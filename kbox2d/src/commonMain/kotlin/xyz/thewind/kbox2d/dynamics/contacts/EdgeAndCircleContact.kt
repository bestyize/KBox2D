package xyz.thewind.kbox2d.dynamics.contacts

import xyz.thewind.kbox2d.collision.Manifold
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.collision.shapes.EdgeShape
import xyz.thewind.kbox2d.collision.shapes.ShapeType
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.pooling.IWorldPool

class EdgeAndCircleContact(argPool: IWorldPool) : Contact(argPool) {
    override fun init(fA: xyz.thewind.kbox2d.dynamics.Fixture, indexA: Int, fB: xyz.thewind.kbox2d.dynamics.Fixture, indexB: Int) {
        super.init(fA, indexA, fB, indexB)
        assert(m_fixtureA!!.getType() == ShapeType.EDGE)
        assert(m_fixtureB!!.getType() == ShapeType.CIRCLE)
    }

    override fun evaluate(manifold: Manifold, xfA: Transform, xfB: Transform) {
        pool.getCollision().collideEdgeAndCircle(manifold, m_fixtureA!!.getShape() as EdgeShape, xfA, m_fixtureB!!.getShape() as CircleShape, xfB)
    }
}
