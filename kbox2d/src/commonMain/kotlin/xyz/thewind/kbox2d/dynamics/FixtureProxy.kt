package xyz.thewind.kbox2d.dynamics

import xyz.thewind.kbox2d.collision.AABB

class FixtureProxy {
    val aabb: AABB = AABB()
    var fixture: Fixture? = null
    var childIndex: Int = 0
    var proxyId: Int = 0
}
