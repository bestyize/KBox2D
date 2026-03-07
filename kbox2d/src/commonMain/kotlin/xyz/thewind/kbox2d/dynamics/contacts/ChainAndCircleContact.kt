package xyz.thewind.kbox2d.dynamics.contacts

import xyz.thewind.kbox2d.collision.Manifold
import xyz.thewind.kbox2d.collision.shapes.ChainShape
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.collision.shapes.EdgeShape
import xyz.thewind.kbox2d.collision.shapes.ShapeType
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.pooling.IWorldPool

class ChainAndCircleContact(argPool: IWorldPool) : Contact(argPool) {
    private val edge = EdgeShape()

    override fun init(fA: xyz.thewind.kbox2d.dynamics.Fixture, indexA: Int, fB: xyz.thewind.kbox2d.dynamics.Fixture, indexB: Int) {
        super.init(fA, indexA, fB, indexB)
        assert(m_fixtureA!!.getType() == ShapeType.CHAIN)
        assert(m_fixtureB!!.getType() == ShapeType.CIRCLE)
    }

    override fun evaluate(manifold: Manifold, xfA: Transform, xfB: Transform) {
        val chain = m_fixtureA!!.getShape() as ChainShape
        chain.getChildEdge(edge, m_indexA)
        pool.getCollision().collideEdgeAndCircle(manifold, edge, xfA, m_fixtureB!!.getShape() as CircleShape, xfB)
    }
}
