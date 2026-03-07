package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Vec2

class Jacobian {
    val linearA: Vec2 = Vec2()
    var angularA: Float = 0f
    var angularB: Float = 0f
}
