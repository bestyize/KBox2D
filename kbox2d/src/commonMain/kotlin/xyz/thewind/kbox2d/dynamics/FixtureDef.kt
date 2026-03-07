package xyz.thewind.kbox2d.dynamics

import xyz.thewind.kbox2d.collision.shapes.Shape

class FixtureDef {
    var shape: Shape? = null
    var userData: Any? = null
    var friction: Float = 0.2f
    var restitution: Float = 0f
    var density: Float = 0f
    var isSensor: Boolean = false
    var filter: Filter = Filter()
}
