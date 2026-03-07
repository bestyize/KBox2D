package xyz.thewind.kbox2d.callbacks

import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Fixture

fun interface RayCastCallback {
    fun reportFixture(fixture: Fixture, point: Vec2, normal: Vec2, fraction: Float): Float
}
