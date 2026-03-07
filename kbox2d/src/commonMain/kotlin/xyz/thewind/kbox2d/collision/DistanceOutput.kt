package xyz.thewind.kbox2d.collision

import xyz.thewind.kbox2d.common.Vec2

class DistanceOutput {
    val pointA: Vec2 = Vec2()
    val pointB: Vec2 = Vec2()
    var distance: Float = 0f
    var iterations: Int = 0
}
